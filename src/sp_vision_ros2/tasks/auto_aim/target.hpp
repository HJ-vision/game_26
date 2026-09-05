#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, double l0 = 0.0, double h0 = 0.0);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  // 只推进时间戳，不做状态预测（用于检测间隙，防止 EKF 纯外推漂移）
  void touch_time(std::chrono::steady_clock::time_point t) { t_ = t; }

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  // ---- 可视化调试访问器 ----
  // 旋转中心（整车）世界坐标，对应状态 [x, y, z] = [ekf_x[0], ekf_x[2], ekf_x[4]]
  Eigen::Vector3d center_xyz() const { return {ekf_.x[0], ekf_.x[2], ekf_.x[4]}; }
  // 旋转半径（状态 r = ekf_x[8]）
  double radius() const { return ekf_.x[8]; }
  // 装甲板数量（标准步兵 4，前哨站/基地 3，平衡步兵 2）
  int armor_num() const { return armor_num_; }

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  double radius_prior_ = 0.2;  // 旋转半径物理先验（标准步兵 ~0.2m），用于 r 硬限幅兜底
  int switch_count_;
  int update_count_;
  int pending_switch_id_ = -1;
  int pending_switch_count_ = 0;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  // 【重构】把 predict() 里的 F / Q 计算抽成成员方法，便于后续扩展（peek_prediction
  // 等需要复用同一组 F / Q 时无需重复维护 60+ 行矩阵定义）。
  Eigen::MatrixXd transition_F(double dt) const;
  Eigen::MatrixXd process_Q(double dt) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP