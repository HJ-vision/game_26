#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  // 带默认值兜底：旧配置缺键不崩溃
  extrapolation_ref_speed_ =
    yaml["extrapolation_ref_speed"] ? yaml["extrapolation_ref_speed"].as<double>() : 1.0;
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  // 初始化阶段先让 EKF 累积观测，避免初始位置/速度误差直接驱动云台。
  if (!target.convergened()) {
    tools::logger()->debug("[AIM-CMD] not converged, no command");
    return {false, false, 0, 0};
  }

  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  // ---------------------------------------------------------------------
  // 【静止靶摆头修复】按目标速度缩放预测外推 horizon
  //
  // 完整外推 horizon = 处理延迟(图像→now，det 40~70ms) + 发弹延时 + 弹丸
  // 飞行时间(~0.3s @5m/23m/s)，合计 ~0.3-0.4s。EKF 速度噪声 σ_v 会被整个
  // horizon 放大成瞄准点横向抖动：σ_yaw ≈ σ_v × T / 距离。静止靶上 EKF
  // 速度纯是噪声（Q 的加速度方差 v1=100 很松），0.3m/s 噪声 × 0.35s / 5m
  // ≈ 1.2°/帧 的指令抖动 → 云台左右摆头。
  //
  // 方案：等效速度 v_eff = 整车平移速度 + |vyaw|×旋转半径（板面切向速度），
  // 外推时间按 (v_eff / ref)² 缩放：
  //   v_eff ≈ 0（静止，只剩噪声）→ 外推≈0，指令基本冻结在滤波位置
  //   v_eff ≥ ref               → 完整外推，动态目标提前量不损失
  // 二次缩放让"噪声速度带"(<0.3m/s) 衰减得比线性更快。
  // ---------------------------------------------------------------------
  Eigen::VectorXd ekf_x0 = target.ekf_x();
  double v_eff =
    std::hypot(ekf_x0[1], ekf_x0[3]) + std::abs(ekf_x0[7]) * std::min(std::abs(ekf_x0[8]), 0.6);
  double speed_scale = std::min(1.0, v_eff / extrapolation_ref_speed_);
  speed_scale *= speed_scale;
  auto cap_horizon = [this, speed_scale](double t_full) { return t_full * speed_scale; };

  // 考虑 detector/tracker 消耗的时间（to_now=false 时按经验值 5ms），
  // 此外假设 aimer 自身用时可忽略不计
  double latency = to_now ? tools::delta_time(std::chrono::steady_clock::now(), timestamp)
                          : 0.005;
  double t_full0 = latency + delay_time;
  auto future =
    timestamp + std::chrono::microseconds(int(cap_horizon(t_full0) * 1e6));
  target.predict(future);

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 timestamp + 限幅后(fly_time) 时刻的位置。
    // 原代码 future + fly_time = 完整 horizon 外推；现在整段 horizon 都按
    // speed_scale 缩放，与初始外推保持一致（缩放比例相同，无内部跳变）。
    double t_full_iter = t_full0 + prev_fly_time;
    auto predict_time =
      timestamp + std::chrono::microseconds(static_cast<int>(cap_horizon(t_full_iter) * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw_raw = std::atan2(final_xyz.y(), final_xyz.x());
  double yaw = yaw_raw + yaw_offset_;
  double pitch = (current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负

  // [AIM-CMD] 指令流水（debug 级，只写日志文件不刷终端）：
  // 静止靶摆头诊断核心数据。看什么：
  //   - v/vyaw 抖不抖：EKF 速度噪声的大小（静止靶应为 0 附近的窄幅噪声）
  //   - T_eff 是否 ≈ 0：速度缩放外推是否生效（静止靶应被压到几十 ms 以下）
  //   - yaw 逐帧变化量：最终下发指令的抖动幅度（deg）
  tools::logger()->debug(
    "[AIM-CMD] v=({:.2f},{:.2f}) vyaw={:.2f} v_eff={:.2f} scale={:.2f} T_full={:.3f} "
    "T_eff={:.3f} fly={:.3f} aim=({:.2f},{:.2f},{:.2f}) yaw={:.2f} pitch={:.2f}",
    ekf_x0[1], ekf_x0[3], ekf_x0[7], v_eff, speed_scale, t_full0 + prev_fly_time,
    cap_horizon(t_full0 + prev_fly_time), current_traj.fly_time, final_xyz.x(), final_xyz.y(),
    final_xyz.z(), yaw * 180.0 / M_PI, pitch * 180.0 / M_PI);

  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  // 【修复】原判断用 x[8]=旋转半径（恒 ~0.2m ≤ 2，永远为真）→ 小陀螺分支
  // 从未生效，旋转靶一直走"跟单板"逻辑（云台追着板摆 ±60°），而不是反陀螺
  // 选板逻辑。x[7] 才是整车角速度 vyaw（rad/s），>2 rad/s ≈ 每 0.8s 换一块板。
  if (std::abs(target.ekf_x()[7]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim