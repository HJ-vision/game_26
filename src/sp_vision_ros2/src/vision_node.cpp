// ============================================================
// vision_node.cpp
// 同济 sp_vision_25 视觉自瞄算法 ROS2 化节点
//
// 两种运行模式：
//   mode = "camera"  实机模式：订阅 /image_raw + /Vision_data（IMU/弹速/模式）
//                    发布 /Robot_ctrl_data（云台控制）+ /aim_debug_image（调试图）
//   mode = "video"   离线验证模式：读视频文件逐帧跑 detect->track->aim，
//                    不依赖 IMU（用单位四元数），适合先在电脑上验证识别效果
//
// 流水线与同济 src/standard.cpp 完全对齐：
//   solver.set_R_gimbal2world(q) -> detector.detect(img) -> tracker.track(armors)
//   -> aimer.aim(targets, t, bullet_speed) -> 输出控制指令
//
// 单位约定：
//   aimer 输出 yaw/pitch 为弧度（rad），队内 STM32 期望角度（deg），发布前转换。
//   target_lock: 49=锁定, 50=未锁定（protocol_new.hpp）
//   fire_command: 1=开火, 0=停火（队内旧代码约定）
//   /Vision_data 四元数顺序为 [w, x, y, z]
// ============================================================

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <auto_aim_interfaces/msg/vision.hpp>
#include <auto_aim_interfaces/msg/robot_ctrl.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "io/command.hpp"

using namespace std::chrono_literals;

namespace sp_vision_ros2
{

class VisionNode : public rclcpp::Node
{
public:
  VisionNode() : Node("vision_node")
  {
    // ---------- 参数 ----------
    config_path_ = declare_parameter<std::string>("config_path", "configs/standard3.yaml");
    mode_ = declare_parameter<std::string>("mode", "camera");
    video_path_ = declare_parameter<std::string>("video_path", "");
    bullet_speed_ = declare_parameter<double>("bullet_speed", 23.0);
    auto_fire_ = declare_parameter<bool>("auto_fire", true);
    debug_ = declare_parameter<bool>("debug", true);
    show_ = declare_parameter<bool>("show", false);
    save_path_ = declare_parameter<std::string>("save_path", "");
    debug_force_yaw_ = declare_parameter<bool>("debug_force_yaw", false);
    debug_ekf_viz_ = declare_parameter<bool>("debug_ekf_viz", true);

    if (debug_force_yaw_) {
      RCLCPP_WARN(get_logger(), "[vision] debug_force_yaw enabled: Robot_ctrl_data.yaw fixed to 1.0 deg for smooth-move test");
    }

    // ---------- 工作目录处理 ----------
    // 同济代码里的模型路径是相对项目根的（assets/xxx.xml），
    // 这里把工作目录切到 config 文件所在目录的上一级（= 项目根），
    // 保证从任何地方启动都能找到模型。
    std::filesystem::path cfg(config_path_);
    if (cfg.is_relative()) {
      cfg = std::filesystem::absolute(cfg);
    }
    std::filesystem::current_path(cfg.parent_path().parent_path());
    RCLCPP_INFO(get_logger(), "[vision] workdir -> %s", std::filesystem::current_path().c_str());

    // ---------- 同济算法核心（与 standard.cpp 一致） ----------
    detector_ = std::make_unique<auto_aim::YOLO>(config_path_, false);
    solver_ = std::make_unique<auto_aim::Solver>(config_path_);
    tracker_ = std::make_unique<auto_aim::Tracker>(config_path_, *solver_);
    aimer_ = std::make_unique<auto_aim::Aimer>(config_path_);
    RCLCPP_INFO(get_logger(), "[vision] algorithm modules loaded (YOLO/Solver/Tracker/Aimer)");

    // ---------- 火控门控参数（从算法配置 yaml 直读，缺键用默认值兜底） ----------
    // 注意：这些键此前只存在于 yaml 中、没有任何代码消费（死键），开火逻辑
    // 一直是"锁定即 fire_command=1"。现在由 publish_ctrl 真正使用它们：
    //   first_tolerance  近距离开火角度容差（degree）
    //   second_tolerance 远距离开火角度容差（degree）
    //   judge_distance   远近距离分界（m）
    //   fire_gap_time    两波开火之间的最小冷却间隔（s）
    try {
      auto yaml = YAML::LoadFile(config_path_);
      if (yaml["first_tolerance"]) first_tolerance_ = yaml["first_tolerance"].as<double>();
      if (yaml["second_tolerance"]) second_tolerance_ = yaml["second_tolerance"].as<double>();
      if (yaml["judge_distance"]) judge_distance_ = yaml["judge_distance"].as<double>();
      if (yaml["fire_gap_time"]) fire_gap_time_ = yaml["fire_gap_time"].as<double>();
      RCLCPP_INFO(
        get_logger(),
        "[vision] fire gating: near_tol=%.1fdeg far_tol=%.1fdeg judge_dist=%.1fm gap=%.2fs",
        first_tolerance_, second_tolerance_, judge_distance_, fire_gap_time_);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "[vision] fire gating params load failed (%s), using defaults", e.what());
    }

    // ---------- 发布器 ----------
    ctrl_pub_ = create_publisher<auto_aim_interfaces::msg::RobotCtrl>("/Robot_ctrl_data", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/aim_debug_image", 1);

    // ---------- 模式分支 ----------
    if (mode_ == "camera") {
      // [FIX] 把 Vision_data 订阅放到独立回调组，避免被 image_cb 的 80ms YOLO 推理阻塞丢帧
      // 现象：电控发 100Hz，但 vision_node 日志 avg_interval≈198ms(5Hz)
      // 根因：两个 sub 在同一 group 且 vision queue_depth=1，image_cb 占 80ms 内来的 8 条直接扔 7 条
      auto vision_cb_group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      auto vision_sub_opt = rclcpp::SubscriptionOptions();
      vision_sub_opt.callback_group = vision_cb_group;

      img_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { image_cb(msg); });
      // queue_depth 10：Vision_data 100Hz → 10条足够缓存 100ms 抖动，配合下游 0.5s 环形缓冲绝不丢
      vision_sub_ = create_subscription<auto_aim_interfaces::msg::Vision>(
        "/Vision_data", 10,
        [this](const auto_aim_interfaces::msg::Vision::ConstSharedPtr msg) { vision_cb(msg); },
        vision_sub_opt);
      RCLCPP_INFO(get_logger(), "[vision] camera mode: sub /image_raw + /Vision_data (Vision_data on independent cb group)");
    } else if (mode_ == "video") {
      if (video_path_.empty()) {
        RCLCPP_ERROR(get_logger(), "[vision] video mode requires 'video_path' parameter!");
        throw std::runtime_error("video_path is empty");
      }
      cap_.open(video_path_);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "[vision] cannot open video: %s", video_path_.c_str());
        throw std::runtime_error("open video failed: " + video_path_);
      }
      timer_ = create_wall_timer(33ms, [this]() { video_step(); });
      RCLCPP_INFO(get_logger(), "[vision] video mode: %s", video_path_.c_str());

      // 可选：保存带检测框的处理后视频到 save_path
      if (!save_path_.empty()) {
        int fps = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));
        if (fps <= 0) fps = 30;
        int vw = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int vh = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        writer_.open(save_path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                     fps, cv::Size(vw, vh));
        if (!writer_.isOpened()) {
          RCLCPP_ERROR(get_logger(), "[vision] cannot open save file: %s", save_path_.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "[vision] saving debug video -> %s (%dx%d @%dfps)",
                      save_path_.c_str(), vw, vh, fps);
        }
      }
    } else {
      RCLCPP_ERROR(get_logger(), "[vision] unknown mode '%s' (camera|video)", mode_.c_str());
      throw std::runtime_error("unknown mode");
    }
  }

private:
  // 带时间戳的 IMU 四元数样本，用于按图像拍摄时刻匹配姿态
  struct ImuSample
  {
    int64_t stamp_ns;
    Eigen::Quaterniond q;
  };

  // ==========================================================
  // camera 模式：图像回调（帧驱动，对齐 standard.cpp 同步流水线）
  // ==========================================================
  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "[vision] cv_bridge exception: %s", e.what());
      return;
    }

    // IMU 四元数（w,x,y,z）：按图像时间戳匹配拍摄时刻的云台姿态，
    // 避免云台运动时"用处理时刻的 IMU 配旧图像"产生瞄准尖峰。
    // 未收到 /Vision_data 前用单位四元数。
    int64_t img_stamp_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
    Eigen::Quaterniond q = imu_received_ ? lookup_q(img_stamp_ns) : Eigen::Quaterniond::Identity();
    solver_->set_R_gimbal2world(q);

    // 主流水线
    // [FIX-dt] 帧时间改为"图像拍摄时刻"（msg->header.stamp 经首帧锚点映射到
    // steady_clock 域）。原实现用检测完成后的 now()，det=60ms 级波动会直接
    // 混进 EKF 的 dt，速度量被 1/dt 放大成锯齿。aimer 的 to_now 外推从该时刻
    // 起算，"拍摄→处理→发射"的延迟自动计入提前量，方向正确。
    // 时间戳缺失/回退时退回旧行为（now()），保证不会拿负 dt 喂 EKF。
    if (!img_time_anchor_set_ && img_stamp_ns > 0) {
      img_time_anchor_ns_ = img_stamp_ns;
      steady_anchor_ = std::chrono::steady_clock::now();
      img_time_anchor_set_ = true;
    }
    bool stamp_ok = img_time_anchor_set_ && img_stamp_ns > 0 && img_stamp_ns >= last_img_stamp_ns_;
    auto t_start = std::chrono::steady_clock::now();
    auto armors = detector_->detect(bgr, frame_count_++);
    auto frame_time = stamp_ok ? steady_anchor_ + std::chrono::nanoseconds(img_stamp_ns - img_time_anchor_ns_)
                               : std::chrono::steady_clock::now();
    if (img_stamp_ns > last_img_stamp_ns_) last_img_stamp_ns_ = img_stamp_ns;
    auto t_after_det = std::chrono::steady_clock::now();
    auto targets = tracker_->track(armors, frame_time);
    auto t_after_trk = std::chrono::steady_clock::now();

    double speed = imu_received_ ? latest_bullet_speed_ : bullet_speed_;
    io::Command cmd = aimer_->aim(targets, frame_time, speed);
    auto t_after_aim = std::chrono::steady_clock::now();
    auto t_total_us =
      std::chrono::duration<double, std::micro>(t_after_aim - t_start).count();

    // 每 30 帧汇总一次各阶段平均耗时（单位 ms），肉眼即可定位瓶颈：
    //   det_ms  = YOLO + 灯条轮廓 + 分类
    //   trk_ms  = solvePnP + optimize_yaw(140次projectPoints) + EKF 更新
    //   aim_ms  = 弹道迭代 10 次
    static double sum_det = 0, sum_trk = 0, sum_aim = 0, sum_tot = 0;
    static int profile_cnt = 0;
    sum_det += std::chrono::duration<double, std::milli>(t_after_det - t_start).count();
    sum_trk += std::chrono::duration<double, std::milli>(t_after_trk - t_after_det).count();
    sum_aim += std::chrono::duration<double, std::milli>(t_after_aim - t_after_trk).count();
    sum_tot += t_total_us / 1000.0;
    if (++profile_cnt >= 30) {
      RCLCPP_INFO(
        get_logger(),
        "[PERF] avg over %d frames: det=%.1f trk=%.1f aim=%.1f total=%.1f ms | ~%.1f Hz",
        profile_cnt, sum_det / profile_cnt, sum_trk / profile_cnt,
        sum_aim / profile_cnt, sum_tot / profile_cnt,
        1000.0 / (sum_tot / profile_cnt));
      sum_det = sum_trk = sum_aim = sum_tot = 0;
      profile_cnt = 0;
    }

    // 新协议 VisionData 不再携带 mode 字段，自瞄模式改为常开：有目标且解算有效即输出控制
    bool active = true;
    publish_ctrl(cmd, targets.empty(), active, active);
    if (debug_) publish_debug(bgr, armors, targets);
  }

  // ==========================================================
  // video 模式：定时器逐帧处理，无 IMU，验证识别/解算效果
  // ==========================================================
  void video_step()
  {
    cv::Mat bgr;
    if (!cap_.read(bgr)) {
      // 视频播完从头再来
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      if (!cap_.read(bgr)) {
        RCLCPP_WARN(get_logger(), "[vision] video ended, stop");
        timer_->cancel();
        return;
      }
    }

    solver_->set_R_gimbal2world(Eigen::Quaterniond::Identity());

    auto armors = detector_->detect(bgr, frame_count_++);
    auto frame_time = std::chrono::steady_clock::now();
    auto targets = tracker_->track(armors, frame_time);

    io::Command cmd = aimer_->aim(targets, frame_time, bullet_speed_);

    publish_ctrl(cmd, targets.empty(), true, true);
    if (debug_) publish_debug(bgr, armors, targets);

    if (writer_.isOpened() && !debug_frame_.empty()) writer_.write(debug_frame_);
    if (show_) {
      cv::imshow("vision_debug", debug_ ? debug_frame_ : bgr);
      if (cv::waitKey(1) == 27) {  // ESC 退出
        timer_->cancel();
        cv::destroyAllWindows();
      }
    }
  }

  // ==========================================================
  // 输出映射：io::Command -> auto_aim_interfaces/RobotCtrl
  // ==========================================================
  void publish_ctrl(
    const io::Command & cmd, bool no_target, bool active, bool keep_angle)
  {
    auto msg = std::make_unique<auto_aim_interfaces::msg::RobotCtrl>();

    if (debug_force_yaw_) {
      // 调试专用：强制云台只发布 yaw=1.0 deg，观察是否平滑移动
      msg->yaw = 1.0f;
      msg->pitch = 0.0f;
      msg->target_lock = 49;
      msg->fire_command = 0;
      msg->yaw_vel = 0.0f;
      msg->yaw_acc = 0.0f;
      msg->pitch_vel = 0.0f;
      msg->pitch_acc = 0.0f;
      ctrl_pub_->publish(std::move(msg));
      return;
    }

    if (active && !no_target && cmd.control) {
      // 有目标且解算有效：直接输出瞄准角度（rad -> deg）
      // 平滑/限幅交由电控端负责，视觉侧不做二次平滑，避免叠加相位滞后
      // 加剧反馈环振荡；单帧垃圾观测已在 EKF 创新量门限 + Tracker 重捕获
      // 校验里拦掉，指令本身不会再出现单帧大跳。
      msg->yaw = static_cast<float>(cmd.yaw * 180.0 / M_PI);
      msg->pitch = static_cast<float>(cmd.pitch * 180.0 / M_PI);
      msg->target_lock = 49;          // 锁定

      // ---------- 视觉侧开火门控 ----------
      // 之前这里是无条件 fire_command = auto_fire_，"瞄上就喊打"。
      // 现在加两层闸门（电控侧若有自己的判断，双重门控只会更保守不会误射）：
      //   1) 对准判定：云台实际角度(IMU反馈)与瞄准指令的偏差 < 距离相关容差
      //      近距离(<judge_distance)用 first_tolerance，远距离用 second_tolerance
      //   2) 冷却判定：上一波开火结束(失准)后需等待 fire_gap_time 才允许再次开火
      // 波次语义：对准 → fire_command=1 持续输出（电控按电平响应）；
      //           失准 → 立即停火并进入冷却。冷却计时以最后一次输出开火为准。
      msg->fire_command = 0;
      if (auto_fire_ && imu_received_) {
        const auto now = std::chrono::steady_clock::now();
        bool aligned = false;
        double e_yaw = 999.0, e_pitch = 999.0, dist = 0.0, tol = 0.0;
        const auto & ap = aimer_->debug_aim_point;
        if (ap.valid) {
          dist = std::hypot(ap.xyza[0], ap.xyza[1]);
          tol = dist < judge_distance_ ? first_tolerance_ : second_tolerance_;
          const double cmd_yaw_deg = cmd.yaw * 180.0 / M_PI;
          const double cmd_pitch_deg = cmd.pitch * 180.0 / M_PI;
          // yaw 需处理 ±180° 环绕
          double dy = cmd_yaw_deg - static_cast<double>(latest_yaw_);
          while (dy > 180.0) dy -= 360.0;
          while (dy < -180.0) dy += 360.0;
          e_yaw = std::abs(dy);
          e_pitch = std::abs(cmd_pitch_deg - static_cast<double>(latest_pitch_));
          aligned = (e_yaw < tol) && (e_pitch < tol);
        }
        // 每帧门控状态（debug 级，进日志文件不刷终端），用于调容差时看误差分布
        RCLCPP_DEBUG(
          get_logger(),
          "[FIRE] aligned=%d e_yaw=%.2f e_pitch=%.2f tol=%.1f dist=%.2f latched=%d",
          static_cast<int>(aligned), e_yaw, e_pitch, tol, dist, static_cast<int>(fire_latched_));
        if (aligned) {
          if (!fire_latched_) {
            // 冷却结束（或从未开过火）→ 开新一波
            const bool cooled =
              !has_fired_ ||
              std::chrono::duration<double>(now - last_fire_time_).count() >= fire_gap_time_;
            if (cooled) {
              fire_latched_ = true;
              RCLCPP_INFO(
                get_logger(), "[FIRE] burst start: e_yaw=%.2f e_pitch=%.2f tol=%.1f dist=%.2fm",
                e_yaw, e_pitch, tol, dist);
            }
          }
          if (fire_latched_) last_fire_time_ = now;  // 冷却从最后一次开火帧起算
        } else {
          fire_latched_ = false;  // 失准立即停火
        }
        msg->fire_command = fire_latched_ ? 1 : 0;
      }
    } else {
      // 无目标 / 不在自瞄模式：不锁不射
      fire_latched_ = false;  // 丢目标即终止开火波次，重锁后重新过冷却检查
      if (keep_angle && imu_received_) {
        // 用 IMU 反馈的当前角度保持云台不动
        msg->yaw = latest_yaw_;
        msg->pitch = latest_pitch_;
      } else {
        msg->yaw = 0.0f;
        msg->pitch = 0.0f;
      }
      msg->target_lock = 50;          // 未锁定
      msg->fire_command = 0;
    }

    // 速度/加速度前馈：先留 0，后续可以从 Tracker 预测状态补
    msg->yaw_vel = 0.0f;
    msg->yaw_acc = 0.0f;
    msg->pitch_vel = 0.0f;
    msg->pitch_acc = 0.0f;

    ctrl_pub_->publish(std::move(msg));
  }

  // ==========================================================
  // 调试图像：画装甲板框 + 类型标签 + EKF 预测位置，发布 /aim_debug_image
  // ==========================================================
  void publish_debug(
    const cv::Mat & bgr, const std::list<auto_aim::Armor> & armors,
    const std::list<auto_aim::Target> & targets)
  {
    cv::Mat vis = bgr.clone();
    for (const auto & a : armors) {
      // 装甲板四个角点连线（顺序：左上->右上->右下->左下），用于检查角点精度
      if (a.points.size() == 4) {
        for (int i = 0; i < 4; i++) {
          cv::line(vis, a.points[i], a.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
          cv::circle(vis, a.points[i], 3, cv::Scalar(255, 0, 0), -1);  // 蓝色角点
        }
      }
      // 中心点（观测）
      cv::circle(vis, a.center, 3, cv::Scalar(0, 0, 255), -1);
      // 标签：类型+名字+置信度
      std::string label;
      if (a.name != auto_aim::not_armor) {
        label = auto_aim::ARMOR_NAMES[a.name] + "/" +
                auto_aim::ARMOR_TYPES[a.type] + "/" +
                std::to_string(a.confidence).substr(0, 4);
      } else {
        label = std::to_string(a.confidence).substr(0, 4);
      }
      cv::putText(
        vis, label, cv::Point(a.box.x, std::max(a.box.y - 5, 5)),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
    }

    // EKF 预测位置可视化（观测已有，这里只叠加卡尔曼滤波预测出的装甲板位置）
    if (debug_ekf_viz_) draw_ekf_viz(vis, targets);

    debug_frame_ = vis;
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vis).toImageMsg();
    debug_pub_->publish(*img_msg);
  }

  // ==========================================================
  // EKF 预测位置可视化：
  //   紫红叉  = 各装甲板预测位置（armor_xyza_list，来自 11 维状态外推）
  //   青色叉  = 整车旋转中心（状态 [x, y, z]）
  //   黄色圆  = 旋转半径轨迹（半径 r，状态第 8 维）
  // 左上角打印 r/yaw/vyaw 等关键状态，用于排查"拟合漂移"。
  // ==========================================================
  void draw_ekf_viz(cv::Mat & vis, const std::list<auto_aim::Target> & targets)
  {
    for (const auto & target : targets) {
      auto x = target.ekf_x();
      auto center = target.center_xyz();
      auto radius = target.radius();

      // --- 旋转中心（青色叉） ---
      auto center_px = solver_->world2pixel(center);
      if (center_px.x >= 0) {
        cv::drawMarker(
          vis, cv::Point(cvRound(center_px.x), cvRound(center_px.y)),
          cv::Scalar(255, 255, 0), cv::MARKER_TILTED_CROSS, 18, 2);
      }

      // --- 旋转半径轨迹（黄色圆，采样 48 点） ---
      constexpr int kCircleSamples = 48;
      std::vector<cv::Point2f> prev_pts;
      for (int k = 0; k <= kCircleSamples; k++) {
        double a = 2.0 * M_PI * k / kCircleSamples;
        // 与 h_armor_xyz 一致：armor = center - r * (cos, sin)
        Eigen::Vector3d pt{center[0] - radius * std::cos(a), center[1] - radius * std::sin(a),
                           center[2]};
        auto px = solver_->world2pixel(pt);
        if (px.x < 0) {
          prev_pts.clear();
          continue;
        }
        cv::circle(vis, cv::Point(cvRound(px.x), cvRound(px.y)), 2,
                   cv::Scalar(0, 255, 255), -1);
        if (!prev_pts.empty()) {
          cv::line(vis, prev_pts.back(), px, cv::Scalar(0, 255, 255), 1);
        }
        prev_pts.push_back(px);
      }

      // --- 各装甲板预测位置（紫红叉 + 编号） ---
      auto xyza_list = target.armor_xyza_list();
      for (int i = 0; i < static_cast<int>(xyza_list.size()); i++) {
        Eigen::Vector3d pred{xyza_list[i][0], xyza_list[i][1], xyza_list[i][2]};
        auto px = solver_->world2pixel(pred);
        if (px.x < 0) continue;
        cv::Point p(cvRound(px.x), cvRound(px.y));
        cv::drawMarker(vis, p, cv::Scalar(255, 0, 255), cv::MARKER_TILTED_CROSS, 16, 2);
        cv::putText(
          vis, "p" + std::to_string(i), p + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          cv::Scalar(255, 0, 255), 1);
      }

      // --- 关键状态文字（左上角） ---
      const auto & ekf_data = target.ekf().data;
      std::vector<std::string> lines{
        "EKF r=" + std::to_string(radius).substr(0, 5),
        "yaw=" + std::to_string(x[6]).substr(0, 6),
        "vyaw=" + std::to_string(x[7]).substr(0, 6),
        "vx=" + std::to_string(x[1]).substr(0, 5) + " vy=" + std::to_string(x[3]).substr(0, 5),
        "cx=" + std::to_string(center[0]).substr(0, 5) + " cy=" + std::to_string(center[1]).substr(0, 5),
        // NIS 门限诊断：阈值 9.488（df=4 上侧 5%）。REJECT=本帧观测被拒，
        // fail_rate=最近窗口被拒比例。速度量乱跳时先看这两行：
        // REJECT 频繁 / fail_rate 高 → 观测被拒后纯外推，速度量锯齿式抖动。
        "NIS=" + std::to_string(ekf_data.count("nis") ? ekf_data.at("nis") : 0.0).substr(0, 6) +
          (ekf_data.count("gate_reject") && ekf_data.at("gate_reject") > 0.5 ? " REJECT!" : ""),
        "fail_rate=" + std::to_string(
                           ekf_data.count("recent_nis_failures") ? ekf_data.at("recent_nis_failures")
                                                                 : 0.0)
                         .substr(0, 5)};
      int y = 30;
      for (const auto & line : lines) {
        cv::putText(
          vis, line, cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1);
        y += 20;
      }
    }
  }

  // ==========================================================
  // /Vision_data 回调：缓存 IMU 四元数、弹速、模式、当前角度
  // 四元数带时间戳存入环形缓冲，供图像回调按拍摄时刻匹配，
  // 避免云台运动时"用处理时刻的 IMU 配旧图像"导致的瞄准尖峰。
  // ==========================================================
  void vision_cb(const auto_aim_interfaces::msg::Vision::ConstSharedPtr msg)
  {
    // protocol_new.hpp：quaternion 顺序为 w,x,y,z
    latest_q_ = Eigen::Quaterniond(
      msg->quaternion[0], msg->quaternion[1], msg->quaternion[2], msg->quaternion[3]);
    latest_bullet_speed_ = msg->shoot_speed;
    latest_mode_ = msg->mode;
    latest_yaw_ = msg->yaw;
    latest_pitch_ = msg->pitch;
    imu_received_ = true;

    // 入队带时间戳的四元数样本（时间用消息头时间戳，无效则退回本地时间）
    ImuSample s;
    s.stamp_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
    if (s.stamp_ns <= 0) {
      s.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // ---------- 诊断：Vision_data 发布频率过低告警 ----------
    // 如果 IMU 发布间隔 > 100ms（<10Hz），姿态插值误差直接放大。
    // 4Hz（250ms）下云台 yaw 以 60°/s 转动时，单次插值误差 ≈ 60°/s * 0.125s = 7.5°。
    // 自瞄帧率目标 30Hz，Vision_data 至少要到 50Hz（间隔 20ms）才够用。
    static int64_t last_imu_stamp = 0;
    static bool imu_rate_warned = false;
    if (last_imu_stamp > 0) {
      int64_t dt_ns = s.stamp_ns - last_imu_stamp;
      if (dt_ns > 100000000LL && !imu_rate_warned) {  // > 100ms = < 10Hz
        RCLCPP_WARN(
          get_logger(),
          "[CLOCK-DIAG] Vision_data publishing too slow! interval=%.1fms (≈%.1fHz). "
          "Target IMU rate is 100~500Hz. Interpolation accuracy will be degraded. "
          "(This message fires once per launch.)",
          dt_ns / 1e6, dt_ns > 0 ? 1e9 / dt_ns : 0.0);
        imu_rate_warned = true;
      }
    }
    last_imu_stamp = s.stamp_ns;

    s.q = latest_q_;
    imu_buf_.push_back(s);
    // 只保留最近 0.5s 的样本，防止无限增长
    while (!imu_buf_.empty() && s.stamp_ns - imu_buf_.front().stamp_ns > 500000000LL) {
      imu_buf_.pop_front();
    }
  }

  // 按图像时间戳在 IMU 缓冲里做插值：
  // 以相机帧为主时钟，时刻 t_image 处的姿态应从两侧最接近的 IMU 样本做 slerp
  // 估计，而不是简单取最近邻样本，避免在采样间隔上产生姿态跳变。
  Eigen::Quaterniond lookup_q(int64_t img_stamp_ns)
  {
    if (imu_buf_.empty()) return latest_q_;
    if (img_stamp_ns <= 0) return latest_q_;

    // ---------- 诊断：IMU 发布频率（每 30 帧打印一次，debug 级只写文件）----------
    static int clk_cnt = 0;
    if (++clk_cnt % 30 == 0 && imu_buf_.size() >= 2) {
      int64_t span = imu_buf_.back().stamp_ns - imu_buf_.front().stamp_ns;
      double avg_interval_ms = static_cast<double>(span) /
                               static_cast<double>(imu_buf_.size() - 1) / 1e6;
      RCLCPP_DEBUG(
        get_logger(),
        "[CLOCK-DIAG] img=%.3f | imu(buf=%zu, avg_interval=%.1fms ≈ %.1fHz) | "
        "imu_range=[%.3f, %.3f]",
        img_stamp_ns / 1e9,
        imu_buf_.size(), avg_interval_ms,
        avg_interval_ms > 0 ? 1000.0 / avg_interval_ms : 0.0,
        imu_buf_.front().stamp_ns / 1e9, imu_buf_.back().stamp_ns / 1e9);
    }

    // ---------- 真正的边界判断：图像戳不在 IMU 缓冲覆盖的时间范围内才 fallback ----------
    // 注意：旧逻辑是"离相邻两个样本都>100ms就fallback"，这在 IMU 发布频率低时
    // （如 4Hz → 间隔 250ms，中间位置两侧差各 125ms）必然误触发，导致每帧都走
    // fallback latest_q_，云台运动时姿态不匹配。
    int64_t buf_start = imu_buf_.front().stamp_ns;
    int64_t buf_end   = imu_buf_.back().stamp_ns;
    constexpr int64_t OUTSIDE_WARN_NS = 200000000LL;  // 超出缓冲 200ms 才打 warn（正常晚到不应有这么大）
    // 节流：断流期间每帧都会命中，不限制会刷几百条；且轻微超出（<200ms，
    // IMU 晚到几个采样周期）属正常现象，不告警
    static int64_t last_fallback_warn_ns = 0;
    int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch()).count();
    auto warn_throttled = [&](const char * tag, bool newer) {
      int64_t delta = newer ? (img_stamp_ns - buf_end) : (buf_start - img_stamp_ns);
      if (delta <= OUTSIDE_WARN_NS) return;  // 轻微超出属正常，不告警
      if (now_ns - last_fallback_warn_ns < 5000000000LL) return;  // 5 秒最多一条
      last_fallback_warn_ns = now_ns;
      RCLCPP_WARN(
        get_logger(),
        "[CLOCK-DIAG] %s: img=%.3f is %.1fms %s than imu=%.3f "
        "-> fallback latest_q (Vision_data stopped? clock skew growing?) "
        "(throttled to 1 msg / 5s)",
        tag, img_stamp_ns / 1e9, delta / 1e6, newer ? "NEWER" : "EARLIER",
        newer ? buf_end / 1e9 : buf_start / 1e9);
    };
    if (img_stamp_ns < buf_start) {
      warn_throttled("OUTDATED", false);
      return imu_buf_.front().q;
    }
    if (img_stamp_ns > buf_end) {
      warn_throttled("NO-FRESH-IMU", true);
      return imu_buf_.back().q;
    }

    // ---------- 正常路径：在缓冲范围内，找最近两点做 slerp ----------
    auto it = std::lower_bound(
      imu_buf_.begin(), imu_buf_.end(), img_stamp_ns,
      [](const ImuSample & s, int64_t ts) { return s.stamp_ns < ts; });

    if (it == imu_buf_.begin()) return it->q;  // 被前面的 range check 拦住了，这里兜底
    if (it == imu_buf_.end())   return imu_buf_.back().q;

    auto prev = std::prev(it);
    auto next = it;
    int64_t t0 = prev->stamp_ns;
    int64_t t1 = next->stamp_ns;
    if (t1 <= t0) return next->q;

    double alpha = static_cast<double>(img_stamp_ns - t0) / static_cast<double>(t1 - t0);
    return prev->q.slerp(alpha, next->q);
  }

  // ---------- 成员 ----------
  std::string config_path_;
  std::string mode_;
  std::string video_path_;
  std::string save_path_;
  double bullet_speed_;
  bool auto_fire_;
  bool debug_;
  bool show_;
  bool debug_force_yaw_;
  bool debug_ekf_viz_;

  // 算法对象
  std::unique_ptr<auto_aim::YOLO> detector_;
  std::unique_ptr<auto_aim::Solver> solver_;
  std::unique_ptr<auto_aim::Tracker> tracker_;
  std::unique_ptr<auto_aim::Aimer> aimer_;

  // ROS 句柄
  rclcpp::Publisher<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr ctrl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Vision>::SharedPtr vision_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 视频
  cv::VideoCapture cap_;
  cv::VideoWriter writer_;

  // 状态
  int frame_count_ = 0;
  bool imu_received_ = false;
  // [FIX-dt] 图像时间戳 → steady_clock 域的首帧锚点。
  // EKF 的 dt 必须用"拍摄时刻"的帧间差，不能用"检测完成时刻"的差
  // （det 耗时波动 ±30% 会直接污染速度量：v ≈ Δp/Δt）。
  bool img_time_anchor_set_ = false;
  int64_t img_time_anchor_ns_ = 0;
  int64_t last_img_stamp_ns_ = 0;
  std::chrono::steady_clock::time_point steady_anchor_;
  Eigen::Quaterniond latest_q_ = Eigen::Quaterniond::Identity();
  double latest_bullet_speed_ = 0.0;
  int latest_mode_ = 0;
  float latest_yaw_ = 0.0f;
  float latest_pitch_ = 0.0f;

  // ---------- 火控门控状态 ----------
  // 波次状态机：aligned → fire_latched_=1 持续输出开火；失准 → 停火 + 冷却
  double first_tolerance_ = 3.0;   // 近距离开火角度容差（deg）
  double second_tolerance_ = 2.0;  // 远距离开火角度容差（deg）
  double judge_distance_ = 2.0;    // 远近距离分界（m）
  double fire_gap_time_ = 0.7;     // 两波开火最小冷却（s）
  bool fire_latched_ = false;      // 当前处于开火波次中
  bool has_fired_ = false;         // 是否开过火（首次不冷却）
  std::chrono::steady_clock::time_point last_fire_time_{};
  cv::Mat debug_frame_;
  std::deque<ImuSample> imu_buf_;  // 带时间戳的 IMU 四元数缓冲
};

}  // namespace sp_vision_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sp_vision_ros2::VisionNode>();
  // [FIX] 双线程 executor：把 Vision_data 回调和图像推理解耦
  // 之前单线程 + image_cb 占 80ms → 100Hz Vision_data 每 80ms 最多处理 1 条（queue=1 只存最新）
  // 结果视觉侧只看到 5Hz，插值门限、时间戳对齐全废。
  // 2 线程就够：一个跑 YOLO/image_cb，一个专门跑 vision_cb 把姿态入缓冲，绝不丢帧。
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions{}, 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
