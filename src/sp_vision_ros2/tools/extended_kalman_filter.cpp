#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
  data["gate_reject"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd P_prior = P;

  // [FIX] 创新量门限（innovation gating）：
  // 原实现先更新 x，再算卡方，垃圾观测已经写进状态，检验形同虚设。
  // 现在先用“先验状态”算创新量和 NIS，超阈值直接拒绝本次观测，
  // 不更新 x / P，防止误检 / 装甲板 id 跳变把位置甩飞。
  Eigen::VectorXd innovation = z_subtract(z, h(x_prior));
  Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  double nis = innovation.transpose() * S.inverse() * innovation;

  // 卡方检验阈值（自由度=4，取置信水平95%）
  // [FIX] 原值 0.711 是卡方分布下侧 5% 分位数，导致正常帧也被判失败。
  // df=4 上侧 5% 临界值为 9.488。
  constexpr double nis_threshold = 9.488;
  constexpr double nees_threshold = 9.488;

  // 无论接受与否都记录 NIS 统计，供 Tracker 判断收敛质量
  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  else data["nis_fail"] = 0;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }
  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = innovation[0];
  data["residual_pitch"] = innovation[1];
  data["residual_distance"] = innovation[2];
  data["residual_angle"] = innovation[3];
  data["nis"] = nis;
  data["recent_nis_failures"] = recent_rate;

  // 观测离先验太远 -> 判定为误检，拒绝更新，状态保持先验
  if (nis > nis_threshold) {
    data["nees"] = 0.0;
    data["nees_fail"] = 0.0;
    data["gate_reject"] = 1.0;  // 标记本帧观测被门限拒绝，供上层诊断
    return x;  // x / P 不变
  }
  data["gate_reject"] = 0.0;

  // 通过门限：正常卡尔曼更新
  Eigen::MatrixXd K = P_prior * H.transpose() * S.inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x_prior, K * innovation);

  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  else data["nees_fail"] = 0;
  data["nees"] = nees;

  return x;
}

}  // namespace tools