#include "soft_force_control_core/payload_estimator.h"

#include <Eigen/Dense>
#include <cmath>

#include "soft_force_control_core/rotation.h"

namespace sfc {

namespace {
// R^2 = 1 - SS_res / SS_tot over all residual components.
double rSquared(const Eigen::VectorXd& y, const Eigen::VectorXd& y_fit) {
  const double ss_res = (y - y_fit).squaredNorm();
  const double ss_tot = (y.array() - y.mean()).matrix().squaredNorm();
  if (ss_tot <= 0) return ss_res <= 1e-12 ? 1.0 : 0.0;
  return 1.0 - ss_res / ss_tot;
}
}  // namespace

void PayloadEstimator::addSample(double a_deg, double b_deg, double c_deg,
                                 const Wrench& averaged) {
  samples_.push_back({a_deg, b_deg, c_deg, averaged});
}

PayloadFitResult PayloadEstimator::solve() const {
  PayloadFitResult res;
  const std::size_t n = samples_.size();
  if (n < 4) return res;

  // Force fit: F_i = -G * r3_i + F0, unknowns x = [G, F0x, F0y, F0z].
  Eigen::MatrixXd af(3 * n, 4);
  Eigen::VectorXd bf(3 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Matrix3d r =
        kukaAbcToRotation(samples_[i].a, samples_[i].b, samples_[i].c);
    const Eigen::Vector3d r3 = r.row(2).transpose();  // R^T * e_z
    af.block<3, 1>(3 * i, 0) = -r3;
    af.block<3, 3>(3 * i, 1) = Eigen::Matrix3d::Identity();
    bf.segment<3>(3 * i) = Eigen::Vector3d(samples_[i].w.fx, samples_[i].w.fy,
                                           samples_[i].w.fz);
  }
  const Eigen::VectorXd xf = af.colPivHouseholderQr().solve(bf);
  if (!xf.allFinite()) return res;

  // Torque fit: T_i = -skew(f_i) * com + T0, f_i = F_meas_i - F0.
  Eigen::MatrixXd at(3 * n, 6);
  Eigen::VectorXd bt(3 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d f(samples_[i].w.fx - xf(1), samples_[i].w.fy - xf(2),
                            samples_[i].w.fz - xf(3));
    Eigen::Matrix3d skew;
    skew << 0, -f.z(), f.y(), f.z(), 0, -f.x(), -f.y(), f.x(), 0;
    at.block<3, 3>(3 * i, 0) = -skew;
    at.block<3, 3>(3 * i, 3) = Eigen::Matrix3d::Identity();
    bt.segment<3>(3 * i) = Eigen::Vector3d(samples_[i].w.tx, samples_[i].w.ty,
                                           samples_[i].w.tz);
  }
  const Eigen::VectorXd xt = at.colPivHouseholderQr().solve(bt);
  if (!xt.allFinite()) return res;

  res.ok = true;
  res.params.gravity_n = xf(0);
  res.params.bias.fx = xf(1);
  res.params.bias.fy = xf(2);
  res.params.bias.fz = xf(3);
  res.params.com_x = xt(0);
  res.params.com_y = xt(1);
  res.params.com_z = xt(2);
  res.params.bias.tx = xt(3);
  res.params.bias.ty = xt(4);
  res.params.bias.tz = xt(5);
  res.r2_force = rSquared(bf, af * xf);
  res.r2_torque = rSquared(bt, at * xt);
  return res;
}

}  // namespace sfc
