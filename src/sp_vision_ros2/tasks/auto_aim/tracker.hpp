#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int detect_lost_count_ = 0;
  int detect_lost_tolerance_;
  // 跟踪阶段观测与预测位置的最大允许距离（米），超过则拒绝该观测
  double max_update_jump_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;

  // 上一次成功跟踪到的目标世界坐标，用于重新捕获时做位置连续性校验
  bool has_last_world_pos_ = false;
  Eigen::Vector3d last_world_pos_ = Eigen::Vector3d::Zero();
  std::chrono::steady_clock::time_point last_track_time_;

  // 重捕获连续拒绝计数：防止 last_world_pos_ 本身是坏值导致永久锁死
  int recapture_reject_count_ = 0;

  // 高度连续变化确认：单帧异常受限，连续同方向变化才放行
  bool has_last_raw_z_ = false;
  double last_raw_z_ = 0.0;
  double last_raw_z_delta_ = 0.0;
  int z_change_count_ = 0;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double dt);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP