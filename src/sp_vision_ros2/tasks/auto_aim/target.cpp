#include "target.hpp"

#include <algorithm>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, double l0, double h0)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  radius_prior_(radius),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1（id 1/3 长轴对相对 id 0/2 的半径差）
  // h: z2 - z1（id 1/3 相对 id 0/2 的高度差）
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, l0, h0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4), radius_prior_(radius)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // [FIX] 状态转移矩阵 z 行去掉 dt*vz 耦合：地面机器人高度基本不变，但
  // solvePnP 的 z 噪声会被匀速模型积分成虚假高度速度，导致目标高度单向漂移
  // （日志里 0.44s 内 z 从 -0.76 漂到 -1.16，pitch 跟着从 3° 抬到 14°，
  // 最终目标掉出画面）。观测模型 H 中 vz 列本来就是 0（z 是直接位置观测），
  // 预测端保留速度耦合只会让漏检外推时高度发散。z 改为只信观测、不外推。
  Eigen::MatrixXd F = transition_F(dt);
  Eigen::MatrixXd Q = process_Q(dt);

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判（仅在 commit 时执行，peek_prediction 不应用这个钳位）
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

Eigen::MatrixXd Target::transition_F(double dt) const
{
  // 状态转移矩阵（11维：[x,vx,y,vy,z,vz,yaw,vyaw,r,l,h]）
  //   行4（z）：{0,0,0,0,1,0,0,0,0,0,0}  —— z 不耦合 vz（见 predict() 注释）
  //   其余：匀速直线 + 匀速旋转（yaw/vyaw）模型
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0, 1,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0, 0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on
  return F;
}

Eigen::MatrixXd Target::process_Q(double dt) const
{
  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    // 【yaw 抖动修复】角加速度方差 400 → 80（详见 predict() 注释 + 工作日志）
    v2 = 80;   // 角加速度方差
  }
  // 【z 独立过程噪声】v3=0.5（详见 predict() 注释 + 工作日志）
  constexpr double v3 = 0.5;
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差（块对角，xy / z / yaw 各自独立）
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v3, b * v3,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v3, c * v3,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on
  return Q;
}

Target::PeekedPrediction Target::peek_prediction(double dt, int id) const
{
  // 占位：peek_prediction 接口保留声明以便未来扩展，目前未使用。
  // 详见工作日志 2026-09-04 §5B.4 马氏门禁的完整实现（在 backup_20260904_mahalanobis/）。
  (void)dt; (void)id;
  return {};
}

void Target::update(const Armor & armor)
{
  // 装甲板匹配
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  // [FIX] 循环上界加 size() 防护：平衡步兵 armor_num=2 时 xyza_i_list 只有
  // 2 个元素，原来的 i<3 会越界读（历史遗留 bug，顺手修掉）。
  double best_pos_err = 0.0;
  double last_id_pos_err = 1e9;  // 当前锁定 id 的位置误差；不在候选中 = 已转走
  const int n_candidates = std::min<int>(3, static_cast<int>(xyza_i_list.size()));
  for (int i = 0; i < n_candidates; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    // 【旋转靶 ID 误匹配修复】原匹配只看两个角度（PnP 装甲板朝向角 + 方位角），
    // 旋转/模糊/远距时 PnP 朝向角噪声可达 ±20~40°，接近 90° 板间距的一半，
    // ID 在相邻板间频繁翻转（pending switch 刷屏）→ 每次切换跳 1~2 帧 EKF 更新
    // → (center,r) 退化解 → r 塌缩到 0 → diverged 重置循环。
    // 加入 xy 平面位置距离项：相邻板位置差 ~r·√2 ≈ 0.37m，远大于位置噪声
    // (~5cm)，让匹配主要信位置、角度只做加权修正。z 不参与（PnP 深度噪声大）。
    constexpr double kPosMatchWeight = 5.0;  // 0.2m 位置差 ≈ 1 rad ≈ 57°
    double pos_err =
      std::hypot(armor.xyz_in_world[0] - xyza[0], armor.xyz_in_world[1] - xyza[1]);
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0])) +
                       kPosMatchWeight * pos_err;

    // 【滞回】当前锁定 id 的误差减去一个偏置：两个候选误差接近时优先保持
    // 当前 id，抑制 PnP 朝向角噪声导致的相邻板来回翻转（pending switch 刷屏）。
    // 偏置 0.3 rad ≈ 17°，远小于真实换板的 ~1.25 rad 位置差，不会挡住正常切换。
    constexpr double kIdHysteresis = 0.3;
    if (xyza_i_list[i].second == last_id) {
      angle_error -= kIdHysteresis;
      last_id_pos_err = pos_err;
    }

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
      best_pos_err = pos_err;
    }
  }

  // 逐帧 ID 选择流水：debug 级只写文件（pending switch 的关键事件保留在终端）
  tools::logger()->debug(
    "[ID-DIAG] selected={} previous={} pending={} pending_count={} update_count={} "
    "match_error={:.2f} pos_err={:.3f} pos_old={:.3f} measured_yaw={:.2f} measured_pitch={:.2f}",
    id, last_id, pending_switch_id_, pending_switch_count_, update_count_,
    min_angle_error * 180.0 / M_PI, best_pos_err, last_id_pos_err,
    armor.ypr_in_world[0] * 180.0 / M_PI,
    armor.ypd_in_world[1] * 180.0 / M_PI);

  int update_id = id;
  bool pending_confirm = false;
  // 【旋转靶】高转速时板间切换是常态而非噪声：3 帧确认（≈200ms @15Hz）
  // 期间板已转 ~0.8 rad，确认时 yaw 残差偏大。|vyaw|>1.5 rad/s 时缩短为 2 帧；
  // 低速/静止时保持 3 帧防噪声误切换（平移靶的 ID 抖动靠这个压住）。
  constexpr int kSwitchConfirmFrames = 3;
  int confirm_frames = std::abs(ekf_.x[7]) > 1.5 ? 2 : kSwitchConfirmFrames;
  // 【位置判决式立即换板】r 锁定后 pos_err 是可靠判据：新板预测距离 <8cm 且
  // 当前板 >15cm（相邻板相距 ~28cm，噪声翻转拉不开这么大差距）= 真实换板，
  // 立即确认，不再等 2-3 帧。低帧率(12-20Hz)下逐帧确认会被 temp_lost 断帧
  // 反复打断（日志实测 (2/3)→(1/3) 循环），换板期 EKF 断粮 → vyaw 估计
  // 远低于真实值（confirm 阈值在 1.5 附近来回切、甚至倒序换板）→
  // 外推 horizon / 反陀螺选板来回变 → 瞄准点距离跳 → 云台点头/摆头。
  constexpr double kDecisiveNewPos = 0.08;
  constexpr double kDecisiveOldPos = 0.15;
  const bool decisive_switch = (update_count_ > 0) && (id != last_id) &&
                               (best_pos_err < kDecisiveNewPos) &&
                               (last_id_pos_err > kDecisiveOldPos);
  if (update_count_ == 0) {
    pending_switch_id_ = -1;
    pending_switch_count_ = 0;
  } else if (decisive_switch) {
    pending_switch_id_ = -1;
    pending_switch_count_ = 0;
    jumped = true;
    tools::logger()->info(
      "[ID-DIAG] decisive switch {}->{} (pos_err {:.2f}->{:.2f}), instant confirm",
      last_id, id, last_id_pos_err, best_pos_err);
  } else if (id == last_id) {
    pending_switch_id_ = -1;
    pending_switch_count_ = 0;
  } else {
    if (pending_switch_id_ == id) {
      pending_switch_count_++;
    } else {
      pending_switch_id_ = id;
      pending_switch_count_ = 1;
    }

    if (pending_switch_count_ < confirm_frames) {
      // [FIX] 候选 ID 尚未稳定。原实现用 last_id 的 h() 去配【新板】的观测，
      // 创新量含板间夹角（4 板车约 90°），NIS 门限必然拒绝，EKF 白白断粮。
      // 改为跳过本帧 EKF 更新（状态停留在 predict 结果），确认后再切换。
      pending_confirm = true;
    } else {
      pending_switch_id_ = -1;
      pending_switch_count_ = 0;
      jumped = true;
    }
  }

  if (pending_confirm) {
    tools::logger()->info(
      "[ID-DIAG] pending switch {}->{} ({}/{}), skip EKF update this frame",
      last_id, id, pending_switch_count_, confirm_frames);
    is_switch_ = false;  // 跳帧期间不算切换，避免残留上一次的值
    return;              // 不更新 EKF、不推进 last_id / update_count_，等候选 ID 确认
  }

  is_switch_ = update_id != last_id;

  if (is_switch_) switch_count_++;

  last_id = update_id;
  update_count_++;

  update_ypda(armor, update_id);
}

void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{
    {4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
     log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);

  // [FIX] r 是已知物理常量（标准步兵 ~0.2m），不是可自由拟合的状态。ID 误配时
  // EKF 会走 r→0 的退化解（四板位置重合、位置残差与 id 无关），r 一旦塌缩，
  // 位置匹配也随之失效，恶性循环。P0 收紧后正常根本不会触发本兜底，这里只在
  // 病态时把 r 硬限幅回物理先验 ±50% 内，作为最后一道保险。
  const double r_lo = std::max(0.05, 0.5 * radius_prior_);
  const double r_hi = std::min(0.8, 1.5 * radius_prior_);
  if (ekf_.x[8] < r_lo || ekf_.x[8] > r_hi) {
    tools::logger()->debug(
      "[Target] clamp r {:.3f} -> {:.3f} (prior {:.3f})",
      ekf_.x[8], std::clamp(ekf_.x[8], r_lo, r_hi), radius_prior_);
    ekf_.x[8] = std::clamp(ekf_.x[8], r_lo, r_hi);
  }

  // 四板车 l/h 物理限幅：旋转靶实测 |l|=0.03（长短轴半径差）、|h|=0.09
  // （高低差）。原 ±0.05 限幅会把 h 卡在 0.05，剩下 4cm 高低差全部漏进
  // 中心 z，以换板频率来回拉扯（上下晃动的直接来源）。放宽到实测范围
  // 外留余量：l ±0.08、h ±0.15。标准步兵 l/h 会收敛到 ~0，不受影响。
  if (armor_num_ == 4) {
    constexpr double kLClamp = 0.08;
    constexpr double kHClamp = 0.15;
    if (std::abs(ekf_.x[9]) > kLClamp) ekf_.x[9] = std::clamp(ekf_.x[9], -kLClamp, kLClamp);
    if (std::abs(ekf_.x[10]) > kHClamp) ekf_.x[10] = std::clamp(ekf_.x[10], -kHClamp, kHClamp);
  }

  // 几何状态流水（debug 级，只写日志文件）：验证 l/h 在线收敛用。
  // 旋转靶期望：|l|→0.03、|h|→0.09（符号取决于先看到哪对板），z 趋稳。
  tools::logger()->debug(
    "[EKF-GEOM] id={} r={:.3f} l={:.3f} h={:.3f} z={:.3f} vyaw={:.2f}", id, ekf_.x[8], ekf_.x[9],
    ekf_.x[10], ekf_.x[4], ekf_.x[7]);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  // 位置合理性：旋转中心必须在相机前方 0.3~30m、高度 |z|<10m
  double px = ekf_.x[0], py = ekf_.x[2], pz = ekf_.x[4];
  double dist = std::sqrt(px * px + py * py);
  if (dist < 0.3 || dist > 30.0 || std::abs(pz) > 10.0) {
    tools::logger()->info("[Target] diverged: center=({:.2f},{:.2f},{:.2f})", px, py, pz);
    return true;
  }

  // [FIX-2] 恢复 r/l 物理范围判发散（在原版 0.05~0.5 基础上放宽到 0.05~0.8，
  // 容纳平衡步兵/哨兵）。之前只查位置，r 拟合出负值（实测跑到 -0.7）也不会
  // 触发重置，EKF 停在退化解上出不来。超范围 -> tracker 判 lost -> 重新初始化。
  double r = ekf_.x[8];
  double rl = ekf_.x[8] + ekf_.x[9];  // 长轴板（id 1/3）用的等效半径
  if (r < 0.05 || r > 0.8 || rl < 0.05 || rl > 0.8) {
    tools::logger()->info("[Target] diverged: r={:.3f}, r+l={:.3f} out of [0.05, 0.8]", r, rl);
    return true;
  }

  return false;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  // [FIX-3] 删掉原 [FIX] 的 r<0.02 兜底：它让 h 在 r 越过 0.02 后变成与 r 无关
  // 的常函数（所有板都返回中心），而 Jacobian H 的 r/yaw 列仍然非零——
  // h 与 H 不一致，EKF 看到永远无法解释的残差，把 r 一路拽到负值（实测 -0.7）。
  // 观测模型必须保持纯净；r 异常由 diverged() 的物理范围判据兜底（触发重置）。
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
