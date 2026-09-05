#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  // 带默认值兜底：旧配置没有该键时用默认 10，避免启动崩溃
  detect_lost_tolerance_ = yaml["detect_lost_tolerance"] ? yaml["detect_lost_tolerance"].as<int>() : 10;
  // 跟踪阶段观测与预测位置的最大允许距离（米），缺省 0.8
  max_update_jump_ = yaml["max_update_jump"] ? yaml["max_update_jump"].as<double>() : 0.8;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.25) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  // 第1步排查：列出本帧所有敌方候选（像素位置 + 编号），判断点头是否来自
  // 「画面中心板还在、底部又多了一块」还是「中心板丢了、只剩底部误检」。
  // debug 级：每帧必发，只写日志文件，不刷终端（要看时 RM_LOG_LEVEL=debug）
  {
    int i = 0;
    tools::logger()->debug("[DETECT-DIAG] state={} n={}", state_, armors.size());
    for (const auto & a : armors) {
      float ux = a.center.x;
      float uy = a.center.y;
      if (a.points.size() == 4) {
        ux = (a.points[0].x + a.points[1].x + a.points[2].x + a.points[3].x) / 4.0f;
        uy = (a.points[0].y + a.points[1].y + a.points[2].y + a.points[3].y) / 4.0f;
      }
      tools::logger()->debug(
        "[DETECT-DIAG] [{}] name={} type={} img=({:.1f},{:.1f}) conf={:.2f}", i++,
        static_cast<int>(a.name), static_cast<int>(a.type), ux, uy, a.confidence);
    }
  }

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t, dt);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->info("[Tracker] Target diverged! -> lost");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->info("[Tracker] Bad Converge (NIS fail) -> lost");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  // [FIX-4] 只有进入稳定 tracking 才记录 last_world_pos_。
  // - detecting：EKF还在收敛过程，写入坏值会影响后续重捕获校验
  // - temp_lost：EKF在纯预测漂移，位置本身就不可靠
  // 用 tracking 状态作为重捕获基准的"黄金快照"。
  if (state_ == "tracking") {
    auto x = target_.ekf_x();
    last_world_pos_ = Eigen::Vector3d(x[0], x[2], x[4]);
    last_track_time_ = t;
    has_last_world_pos_ = true;
  }

  std::list<Target> targets = {target_};
  return targets;
}

void Tracker::state_machine(bool found)
{
  auto prev = state_;

  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
    detect_lost_count_ = 0;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      detect_lost_count_ = 0;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      // [FIX] 原逻辑漏检一帧就清零回 lost，低帧率下极易反复
      // detecting<->lost 死循环。改为允许连续漏检若干帧。
      detect_lost_count_++;
      if (detect_lost_count_ > detect_lost_tolerance_) {
        detect_count_ = 0;
        detect_lost_count_ = 0;
        state_ = "lost";
      }
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }

  // 诊断：打印状态跳变，定位"反复丢锁->重初始化"
  if (state_ != prev) {
    tools::logger()->info("[Tracker] state: {} -> {}", prev, state_);
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);
  if (armor.name == ArmorName::not_armor) return false;

  has_last_raw_z_ = false;
  last_raw_z_delta_ = 0.0;
  z_change_count_ = 0;

  // [FIX] 重新捕获位置连续性校验：
  // 丢锁后短时间内重新检测到的装甲板，如果世界坐标离上次跟踪位置太远，
  // 说明锁到了假目标（误检/另一块板），拒绝初始化，防止云台被甩飞。
  // 允许阈值随丢失时长放宽（目标可能真的在动，按 2m/s 估算）。
  //
  // [FIX-2] 连续 reject 超过 10 次强制放开：防止 last_world_pos_ 本身就是坏值
  //         （比如EKF初始化错了z轴）导致系统永久锁死无法重新捕获。
  if (has_last_world_pos_) {
    auto dt_lost = tools::delta_time(t, last_track_time_);
    if (dt_lost < 3.0) {
      double dx = armor.xyz_in_world[0] - last_world_pos_[0];
      double dy = armor.xyz_in_world[1] - last_world_pos_[1];
      double dz = armor.xyz_in_world[2] - last_world_pos_[2];
      double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
      double max_jump = 0.8 + 2.0 * dt_lost;
      if (jump > max_jump) {
        recapture_reject_count_++;
        if (recapture_reject_count_ < 10) {
          tools::logger()->info(
            "[Tracker] reject re-capture: pos jump {:.2f}m > {:.2f}m (lost {:.2f}s, reject_cnt={})",
            jump, max_jump, dt_lost, recapture_reject_count_);
          return false;
        } else {
          // 连续拒10次，说明 last_world_pos_ 已失效。清掉基准，本帧强制接受。
          tools::logger()->warn(
            "[Tracker] recapture reject x{}, force reset last_world_pos_ (was [{:.2f},{:.2f},{:.2f}])",
            recapture_reject_count_, last_world_pos_[0], last_world_pos_[1], last_world_pos_[2]);
          has_last_world_pos_ = false;
          recapture_reject_count_ = 0;
        }
      } else {
        // 本帧通过了连续性检查，重置计数器
        recapture_reject_count_ = 0;
      }
    }
  }

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    // 平衡步兵：r 是已知物理常量，P0 收紧到 1e-4（不再自由拟合），防止
    // ID 误配时 EKF 走 r→0 退化解导致半径塌缩。
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 1e-4, 1e-4}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    // 标准四板车（含旋转靶）。旋转靶实测几何（2026-09-04）：
    //   短轴对半径 0.21m、长轴对 0.24m（l=0.03）、高低差 h=0.09m。
    // 注意 id0 是"最先看到的那块板"：先看到短轴低板则 l/h 为正，先看到
    // 长轴高板则真实 l/h 反号。因此初值按短轴低板先入锁给出，但 P0 给
    // 0.02（σ=14cm）而非 1e-4 锁死，让 EKF 在 1~2 圈内自动学到正确符号
    // 和幅值；标准步兵会自动收敛回 ~0。
    // r 同理放开到 0.01（σ=10cm）：先看到长轴对时 r 要从 0.21 学到 0.24，
    // 锁死会留 3cm 系统性偏差。r 塌缩退化解由 update_ypda 的硬限幅
    // （先验 ±50% → [0.105, 0.315]）兜底，不依赖 P0 锁死。
    // 【比赛注意】打标准步兵时建议把初值 0.03/0.09 改回 0（收敛更快）。
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.001, 0.001, 0.004}};
    target_ = Target(armor, t, 0.21, 4, P0_dig, 0.0, 0.0);
  }

  return true;
}

bool Tracker::update_target(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double dt)
{
  // 第一遍：只统计同 name/type 的候选（不做 solve，保持轻量）
  int found_count = 0;
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
  }

  // 诊断：检测到了装甲板但编号/大小没匹配上 -> 分类抖动导致丢锁
  // debug 级：分类抖动期间每帧都发，只写文件
  if (found_count == 0 && !armors.empty()) {
    for (const auto & armor : armors) {
      tools::logger()->debug(
        "[Tracker] mismatch: target name={} type={} | detected name={} type={}",
        static_cast<int>(target_.name), static_cast<int>(target_.armor_type),
        static_cast<int>(armor.name), static_cast<int>(armor.type));
    }
  }

  if (found_count == 0) {
    // [FIX] 检测间隙(temp_lost)不做预测外推，只推进时间戳。
    target_.touch_time(t);
    return false;
  }

  // ---------------------------------------------------------------------------
  // [FIX-2] 先不调用 target_.predict()！用 EKF 当前状态做匀速手动外推，
  //         得到一个"虚拟预测位置"作为候选比对基准。
  //         原代码先 predict → 再选候选 → 被拒但 predict 结果已经写入 EKF，
  //         导致每被拒一帧 EKF 就多飞几米（日志里 xyz 从 3m 漂到 6m+ 就是这样）。
  //         现在：只有 best 通过了 jump 门槛，才真正执行 predict + update。
  //         被拒则只 touch_time，状态停留在最后一次成功更新时的位置。
  // ---------------------------------------------------------------------------
  auto x_now = target_.ekf_x();  // EKF 当前状态（未预测）
  // 11维状态约定：[x, vx, y, vy, z, vz, yaw, vyaw, r, ?, ?]
  // z 轴不耦合速度（项目硬约束：z' = z）
  double pred_x = x_now[0] + x_now[1] * dt;
  double pred_y = x_now[2] + x_now[3] * dt;
  double pred_z = x_now[4];  // z 不做速度外推
  Eigen::Vector3d pred_pos(pred_x, pred_y, pred_z);

  // 最优候选选择：只接受"世界坐标离预测位置最近"的那一块
  Armor * best = nullptr;
  double best_dist = 1e10;
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    solver_.solve(armor);
    if (armor.name == ArmorName::not_armor) continue;
    Eigen::Vector3d obs(armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2]);
    double dist = (obs - pred_pos).norm();
    if (dist < best_dist) {
      best_dist = dist;
      best = &armor;
    }
  }

  // ---------------------------------------------------------------------------
  // [FIX-3] 不同状态用不同的 jump 门槛：
  //   - detecting：完全放开（warmup），让 EKF 用真实观测快速收敛
  //   - temp_lost：放宽到 2.5m，给 EKF 从错误位置被"拉回来"的机会
  //   - tracking：保持严格的 0.8m，抗误检/错目标
  // ---------------------------------------------------------------------------
  bool in_detecting_warmup = (state_ == "detecting");
  bool in_temp_lost_recovery = (state_ == "temp_lost");
  double jump_threshold = in_temp_lost_recovery ? (2.5 * max_update_jump_)  // temp_lost: 2.0~2.5m
                                                : max_update_jump_;          // tracking: 0.8m

  if (best == nullptr) {
    tools::logger()->debug(
      "[TRACKER-DIAG] no valid candidate: state={} pred=({:.3f},{:.3f},{:.3f})",
      state_, pred_pos[0], pred_pos[1], pred_pos[2]);
    has_last_raw_z_ = false;
    last_raw_z_delta_ = 0.0;
    z_change_count_ = 0;
    target_.touch_time(t);
    return false;
  }

  // 逐帧观测流水：debug 级只写文件（排查 dt/世界坐标时用）
  tools::logger()->debug(
    "[TRACKER-OBS] state={} dt={:.3f} img_center=({:.1f},{:.1f}) "
    "gim=({:.3f},{:.3f},{:.3f}) "
    "world=({:.3f},{:.3f},{:.3f}) pred=({:.3f},{:.3f},{:.3f}) dist={:.3f}",
    state_, dt,
    (best->points[0].x + best->points[1].x + best->points[2].x + best->points[3].x) / 4.0,
    (best->points[0].y + best->points[1].y + best->points[2].y + best->points[3].y) / 4.0,
    best->xyz_in_gimbal[0], best->xyz_in_gimbal[1], best->xyz_in_gimbal[2],
    best->xyz_in_world[0], best->xyz_in_world[1], best->xyz_in_world[2],
    pred_pos[0], pred_pos[1], pred_pos[2], best_dist);

  if (!in_detecting_warmup && best_dist > jump_threshold) {
    tools::logger()->debug(
      "[Tracker] reject update (state={}): nearest candidate {:.2f}m > {:.2f}m from prediction "
      "-> touch_time (NO predict to prevent drift)",
      state_, best_dist, jump_threshold);
    has_last_raw_z_ = false;
    last_raw_z_delta_ = 0.0;
    z_change_count_ = 0;
    target_.touch_time(t);
    return false;
  }

  constexpr double kMaxSingleFrameZJump = 0.15;
  constexpr double kMinZChangeForTrend = 0.02;
  constexpr int kZChangeConfirmFrames = 3;
  double raw_z = best->xyz_in_world[2];
  double z_delta = raw_z - pred_z;
  double raw_z_delta = has_last_raw_z_ ? raw_z - last_raw_z_ : 0.0;
  bool z_trend_continues = has_last_raw_z_ &&
                           std::abs(raw_z_delta) >= kMinZChangeForTrend &&
                           (z_change_count_ == 0 || raw_z_delta * last_raw_z_delta_ > 0.0);
  if (z_trend_continues) {
    z_change_count_++;
  } else if (has_last_raw_z_ && std::abs(raw_z_delta) < kMinZChangeForTrend) {
    z_change_count_ = 0;
  } else if (has_last_raw_z_) {
    z_change_count_ = 1;
  } else {
    z_change_count_ = 1;
  }
  last_raw_z_ = raw_z;
  last_raw_z_delta_ = raw_z_delta;
  has_last_raw_z_ = true;

  if (std::abs(z_delta) > kMaxSingleFrameZJump && z_change_count_ < kZChangeConfirmFrames) {
    tools::logger()->debug(
      "[Tracker] reject z jump: raw_z={:.3f} pred_z={:.3f} trend={}/{}",
      raw_z, pred_z, z_change_count_, kZChangeConfirmFrames);
    target_.touch_time(t);
    return false;
  }

  // [FIX] 持续性 z 偏差上限：趋势确认只说明"观测自身连续"，不能说明"它就是原目标"。
  // 实机日志实锤：画面底部的可疑检测（世界 z 为负、比真实目标低 0.8~1m）连续 3 帧
  // 骗过趋势确认，之后每帧以 0.15m 的限幅把 EKF z 一路拖向 -0.9，跟踪彻底报废。
  // 真实垂直运动在 15~30Hz 下单帧 z_delta 不会超过 ~0.3m；偏差 >0.5m 且持续存在，
  // 只能是换了个目标（另一辆车/地面反光/斜视角坏解），拒绝并等 temp_lost 超时重捕。
  constexpr double kMaxSustainedZDelta = 0.5;
  if (std::abs(z_delta) > kMaxSustainedZDelta) {
    // 节流：异常目标在场期间每帧命中，前 3 条之后每 30 条提醒一次
    static int sustained_reject_count = 0;
    if (++sustained_reject_count <= 3 || sustained_reject_count % 30 == 0) {
      tools::logger()->warn(
        "[Tracker] reject sustained z offset: raw_z={:.3f} pred_z={:.3f} |delta|={:.2f}m > {:.2f}m "
        "(different target or bad PnP? waiting for temp_lost to expire, cnt={})",
        raw_z, pred_z, z_delta, kMaxSustainedZDelta, sustained_reject_count);
    }
    has_last_raw_z_ = false;
    last_raw_z_delta_ = 0.0;
    z_change_count_ = 0;
    target_.touch_time(t);
    return false;
  }

  Armor update_armor = *best;
  auto ekf_before_update = target_.ekf_x();
  if (std::abs(z_delta) > kMaxSingleFrameZJump) {
    double limited_z = pred_z + std::clamp(
      z_delta, -kMaxSingleFrameZJump, kMaxSingleFrameZJump);
    update_armor.xyz_in_world[2] = limited_z;
    update_armor.ypd_in_world = tools::xyz2ypd(update_armor.xyz_in_world);
  }

  if (in_detecting_warmup && best_dist > max_update_jump_) {
    tools::logger()->debug(
      "[Tracker] detecting warmup: accept best_dist={:.2f}m to seed EKF", best_dist);
  }

  // 只有 best 真正合格，才做 predict + update
  target_.predict(t);
  target_.update(update_armor);
  auto ekf_after_update = target_.ekf_x();
  tools::logger()->debug(
    "[TRACKER-EKF] raw_z={:.3f} pred_z={:.3f} used_z={:.3f} z_delta={:.3f} "
    "before=(z:{:.3f},vz:{:.3f}) after=(z:{:.3f},vz:{:.3f}) limited={}",
    raw_z, pred_z, update_armor.xyz_in_world[2], z_delta,
    ekf_before_update[4], ekf_before_update[5], ekf_after_update[4], ekf_after_update[5],
    std::abs(z_delta) > kMaxSingleFrameZJump ? 1 : 0);
  return true;
}

}  // namespace auto_aim