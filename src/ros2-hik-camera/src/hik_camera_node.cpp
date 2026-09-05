#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list; //一个临时的“花名册”。用来存储电脑上目前插着的所有海康相机的信息
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]); //硬编码选择第一台相机

    nRet = MV_CC_OpenDevice(camera_handle_);
    RCLCPP_INFO(this->get_logger(), "OpenDevice result: [0x%x]", nRet);

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_); //获取图像基本信息，存入img_info_结构体
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3); //预分配图像数据内存，最大支持rgb8格式

    // Init convert param
    //初始化像素格式转换参数
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    rclcpp::QoS qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
    // 只发 raw：不挂 image_transport 的 compressedDepth/theora 子话题，避免 rgb8 每帧报错
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("image_raw/camera_info", qos);

    declareParameters(); //声明并初始化参数

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); // 0=Off 连续采集
    MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2); // 2=Continuous
    // [FIX] 锁最高相机帧率：海康默认可能受曝光/ISP/AE优先级影响跑不满。
    // 启用帧率控制并锁定到 60fps，配合 exposure_time=3ms（≈333fps 上限）
    // 可以稳定输出相机端帧率。如果 AE 生效则此设置可能被忽略，但不报错。
    MV_CC_SetEnumValue(camera_handle_, "AcquisitionFrameRateEnable", 1);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", 60.0f);
    nRet = MV_CC_StartGrabbing(camera_handle_);
    RCLCPP_INFO(this->get_logger(), "StartGrabbing result: [0x%x]", nRet);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));


    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_); //实例化相机信息管理器
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url); //加载相机内参文件
      camera_info_msg_ = camera_info_manager_->getCameraInfo(); //获取并存入相机内参消息
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1)); //注册参数回调函数

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame; //定义一个帧结构体，用于存储每一帧采集到的原始图像的指针和信息

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);// 1. 获取原始数据 (超时时间 1000ms)
                                                                      // 数据指针会存在 out_frame.pBufAddr 中
        if (MV_OK == nRet) {
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_); //转换像素格式 (Bayer 转 RGB)

          // [FIX] 用硬件帧时间戳（秒+微秒合成）保证帧间隔精度，不用 this->now() 直接打戳。
          // 后者是"处理完才打戳"，最大可引入 15-20ms 抖动，直接污染下游 EKF dt，
          // 造成 Q 矩阵 dt⁴ 项膨胀和位置漂移。
          uint64_t hw_sec  = out_frame.stFrameInfo.nSecondCount;
          uint64_t hw_usec = out_frame.stFrameInfo.nCycleCount;  // 通常就是 μs 计数
          int64_t hw_ns = static_cast<int64_t>(hw_sec) * 1000000000LL +
                          static_cast<int64_t>(hw_usec) * 1000LL;

          // [FIX] 时钟域对齐：相机硬件时钟（上电起计）与系统时钟（/Vision_data 用 this->now()）
          // 不在同一纪元，直接发硬件戳会让下游 lookup_q 的 diff>100ms 兜底每帧触发，
          // 姿态插值失效、退回最新四元数，云台运动时世界系坐标跳变（自瞄抬头正反馈的元凶之一）。
          // 这里估计"系统时钟 - 相机时钟"的偏移（含采集/传输延迟，近似常数），
          // 用指数平滑跟踪，再把硬件戳平移到系统时钟域。
          // 这样既保留硬件戳的帧间隔稳定性（EKF dt 准），又能和 IMU 时间戳正确对齐。
          auto now = this->now();
          // rclcpp::Time 直接拿总纳秒（跨 ROS2 版本都对，不要再拆 .sec/.nanosec 那是 msg 结构）
          int64_t sys_ns = now.nanoseconds();
          int64_t stamp_ns;
          if (hw_ns <= 0) {
            // 硬件时间戳异常兜底：直接用系统时间
            stamp_ns = sys_ns;
          } else {
            int64_t offset_est = sys_ns - hw_ns;
            if (!clock_offset_init_) {
              clock_offset_ns_ = static_cast<double>(offset_est);
              clock_offset_init_ = true;
              clock_offset_warmup_ = 0;
              RCLCPP_INFO(
                this->get_logger(),
                "[CLOCK-OFFSET] init: offset=%.3f ms (sys-hw)",
                clock_offset_ns_ / 1e6);
            } else {
              // [OPT] 冷启动前 20 帧用更大的学习率（alpha=0.5）快速收敛：
              // 相机->系统的偏移本质是常数，第一帧的值已经"很接近真实"（只差处理/调度抖动），
              // 但 0.1 平滑需要 20+ 帧才能把初值误差吸收掉（头 2 秒戳不准）。
              // 热启动（第 21 帧起）再切回 alpha=0.1 稳跟踪，吸收两个时钟的微小频差。
              double alpha = (clock_offset_warmup_ < 20) ? 0.5 : 0.1;
              clock_offset_ns_ =
                clock_offset_ns_ * (1.0 - alpha) + static_cast<double>(offset_est) * alpha;
              clock_offset_warmup_++;
              if (clock_offset_warmup_ == 20) {
                RCLCPP_INFO(
                  this->get_logger(),
                  "[CLOCK-OFFSET] converged: offset=%.3f ms (warmup=%d frames)",
                  clock_offset_ns_ / 1e6, clock_offset_warmup_);
              }
            }
            stamp_ns = hw_ns + static_cast<int64_t>(clock_offset_ns_);
          }
          image_msg_.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
          image_msg_.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);

          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width  = out_frame.stFrameInfo.nWidth;
          image_msg_.step   = out_frame.stFrameInfo.nWidth * 3; // 步长：宽 * 3通道,rgb8 每个像素 3 字节
          // [FIX] 只在分辨率变化时才重新分配 data，避免每帧做 vector resize
          const size_t need = static_cast<size_t>(image_msg_.width) *
                              static_cast<size_t>(image_msg_.height) * 3;
          if (image_msg_.data.size() != need) {
            image_msg_.data.resize(need);
          }

          camera_info_msg_.header = image_msg_.header;
          image_pub_->publish(image_msg_);
          camera_info_pub_->publish(camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame); //释放图像缓冲区
          fail_conut_ = 0;
        } else { //采集图像失败
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
  
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      }
      else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      }
      else if (param.get_name() == "balance_ratio_r") {
        int status = MV_CC_SetFloatValue(camera_handle_, "BalanceRatio_R", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set BalanceRatio_R, status = " + std::to_string(status);
        }
      }
      else if (param.get_name() == "balance_ratio_g") {
        int status = MV_CC_SetFloatValue(camera_handle_, "BalanceRatio_G", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set BalanceRatio_G, status = " + std::to_string(status);
        }
      }
      else if (param.get_name() == "balance_ratio_b") {
        int status = MV_CC_SetFloatValue(camera_handle_, "BalanceRatio_B", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set BalanceRatio_B, status = " + std::to_string(status);
        }
      }
      else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
  
      // 一旦失败就可以直接返回，也可以继续检查其它参数：
      if (!result.successful) {
        return result;
      }
    }
  
    return result;
  }
  

  //ROS通信相关
  sensor_msgs::msg::Image image_msg_; //图像消息
  sensor_msgs::msg::CameraInfo camera_info_msg_; //相机内参消息

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_; //只发 raw /image_raw，不挂压缩子话题
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_; //标定所需 camera_info（/image_raw/camera_info）

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_; //负责读取和管理.yaml的相机信息
  
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_; //参数回调句柄，监听参数的动态修改
  
  //hik SDK硬件相关
  int nRet = MV_OK; //SDK函数统一返回值，返回MV_OK（0）表示成功
  void * camera_handle_; //相机句柄,指向要操作的相机
  MV_IMAGE_BASIC_INFO img_info_; //图像基本信息，记录图像宽高等参数，用于申请内存

  MV_CC_PIXEL_CONVERT_PARAM convert_param_; //像素格式转换参数结构体，用于图像格式转换（bayer转rgb）

  //其他
  std::string camera_name_; //相机名称，用于camera_info_manager_加载对应相机内参
  int fail_conut_ = 0; //连续采集失败计数器
  std::thread capture_thread_; //图像采集线程，用于循环采集图像并发布，不干扰ros主线程

  // 相机硬件时钟 -> 系统时钟的偏移估计（时钟域对齐，供下游按图像戳匹配 IMU 姿态）
  bool clock_offset_init_ = false;
  double clock_offset_ns_ = 0.0;
  int clock_offset_warmup_ = 0;  // 冷启动计数：前 20 帧用大步长快速收敛

};
}  // namespace hik_camera


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
