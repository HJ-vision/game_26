#ifndef AUTO_AIM__YOLOV5_TRT_HPP
#define AUTO_AIM__YOLOV5_TRT_HPP

// ============================================================================
// YOLOV5 TensorRT 后端（Jetson GPU 推理）
//
// 模型：0526.onnx（同济系 RM 装甲板模型，yolov5 架构）
//   输入: [1, 3, 640, 640]  NCHW, RGB, /255 归一化, fp16 engine
//   输出: [1, 25200, 22]    25200 anchors × 22 通道
//     通道布局（与 yolov5.cpp 的 parse 完全一致）:
//       [0..7]  = 4 个装甲板角点关键点 (x,y)×4
//       [8]     = objectness（需 sigmoid）
//       [9..12] = 颜色独热（4 类）
//       [13..21]= 编号独热（9 类）
//
// 仅在 CMake 检测到 TensorRT 时编译（SP_VISION_HAS_TENSORRT）。
// engine 构建走 trtexec（/usr/src/tensorrt/bin/trtexec，libnvinfer-bin 自带，
// 无需 onnxparser 头文件），本类只负责：
//   engine 缓存加载（比 onnx 旧自动重建）+ 反序列化失败自动重建
//   + 预处理（blobFromImage: BGR→RGB /255 CHW）
//   + enqueueV3 推理（TRT10 tensor-name API，I/O 名运行时自动发现）
//   + 22 通道 RM 布局后处理（与 OpenVINO 版逐位一致）
//
// 【上次教训（best2-sim 15 通道崩溃）】输出 channel 数 != 22 时直接 throw，
//   不再只 warn —— 通道错配的 parse 是内存越界，不是可降级的软错误。
// ============================================================================

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "NvInfer.h"
#include "cuda_runtime.h"

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

// spdlog -> nvinfer1::ILogger 桥接
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override;
};

class YOLOV5TRT : public YOLOBase
{
public:
  YOLOV5TRT(const std::string & config_path, bool debug);
  ~YOLOV5TRT() override;

  std::list<Armor> detect(const cv::Mat & img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  // ---- engine 生命周期 ----
  // 加载 engine 缓存；不存在 / 比 onnx 旧 / 反序列化失败时调 trtexec 重建
  bool load_or_build_engine();
  bool build_engine_with_trtexec();
  bool deserialize_engine(const std::string & path);

  std::string onnx_path_;   // TRT 构建源（yaml: yolov5_trt_onnx_path）
  std::string engine_path_; // engine 缓存（yaml: yolov5_trt_engine_path）
  std::string save_path_;
  bool debug_, use_roi_, use_traditional_;

  int input_size_;      // 配置期望输入边长（yaml: input_size，须与 engine 一致）
  int trt_input_size_;  // engine 实际输入边长（以 engine 内的 shape 为准）
  // RM 22 通道布局常量（与 OpenVINO 版 yolov5.hpp 对齐）
  static constexpr int kOutputChannels = 22;  // 8 kpt + 1 obj + 4 color + 9 class
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  TrtLogger trt_logger_;
  nvinfer1::IRuntime * runtime_ = nullptr;
  nvinfer1::ICudaEngine * engine_ = nullptr;
  nvinfer1::IExecutionContext * context_ = nullptr;

  // I/O 张量名（运行时从 engine 自动发现，不硬编码）
  std::string input_name_, output_name_;
  int output_c_ = 0;  // 输出通道数（应为 22）
  int output_n_ = 0;  // 锚点数（640 输入时 25200）

  // 推理缓冲（构造时一次性分配，detect 内复用，避免每帧 malloc）
  float * host_input_ = nullptr;
  float * host_output_ = nullptr;
  void * dev_input_ = nullptr;
  void * dev_output_ = nullptr;
  cudaStream_t stream_ = nullptr;
  cv::Mat canvas_;  // 预处理复用的正方形画布（letterbox 底色填充）

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  Detector detector_;  // use_traditional 二次矫正（与 OpenVINO 版一致）
  friend class MultiThreadDetector;

  // ---- 与 YOLOV5（OpenVINO 版）完全一致的后处理 ----
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  std::list<Armor> parse(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);
  void save(const Armor & armor) const;
  void draw_detections(
    const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  static double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_HPP
