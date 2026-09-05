#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

  // 单点世界坐标 -> 像素坐标（供 EKF 预测位置可视化调试）
  // 返回 (-1,-1) 表示该点在相机后方/不可见，调用方应跳过绘制
  cv::Point2f world2pixel(const Eigen::Vector3d & world_point) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  // [最小补丁1-PnP翻解过滤] 无状态重投影像素误差阈值：
  // solvePnP IPPE 返回两解（正面/反面），都满足4个射线方程但角点像素噪声会让反面解反投影误差显著更大。
  // 好解：~2-5px；IPPE翻解/角点定位粗劣：>12px。阈值8px放在中间档。
  // 若超标：armor.name = not_armor 丢弃（后面tracker/update_target流程会忽略它）。
  static constexpr double kMaxReprojErrorPx = 8.0;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP