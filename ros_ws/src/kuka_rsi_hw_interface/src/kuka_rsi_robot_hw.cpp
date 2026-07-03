#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"

#include <cmath>

namespace kuka_rsi {

namespace {
constexpr double kDegToRad = M_PI / 180.0;
}  // namespace

KukaRsiRobotHW::KukaRsiRobotHW() : monitor_(SessionConfig{}) {}

bool KukaRsiRobotHW::configure(const HwConfig& cfg) {
  cfg_ = cfg;
  SessionConfig sc;
  sc.max_consecutive_timeouts = cfg.max_consecutive_timeouts;
  monitor_ = RsiSessionMonitor(sc);
  if (!udp_.bind(cfg.listen_ip, cfg.listen_port)) return false;
  registerInterfaces();
  return true;
}

bool KukaRsiRobotHW::init(ros::NodeHandle& /*root_nh*/,
                          ros::NodeHandle& robot_hw_nh) {
  HwConfig cfg;
  robot_hw_nh.param<std::string>("listen_ip", cfg.listen_ip, cfg.listen_ip);
  int port = cfg.listen_port;
  robot_hw_nh.param("listen_port", port, port);
  cfg.listen_port = static_cast<std::uint16_t>(port);
  robot_hw_nh.param("read_timeout_ms", cfg.read_timeout_ms,
                    cfg.read_timeout_ms);
  int max_timeouts = static_cast<int>(cfg.max_consecutive_timeouts);
  robot_hw_nh.param("max_consecutive_timeouts", max_timeouts, max_timeouts);
  cfg.max_consecutive_timeouts = static_cast<unsigned>(max_timeouts);
  robot_hw_nh.param("max_correction_trans", cfg.limits.max_trans,
                    cfg.limits.max_trans);
  robot_hw_nh.param("max_correction_rot", cfg.limits.max_rot,
                    cfg.limits.max_rot);
  return configure(cfg);
}

void KukaRsiRobotHW::registerInterfaces() {
  static const char* const kJointNames[6] = {"joint_a1", "joint_a2",
                                             "joint_a3", "joint_a4",
                                             "joint_a5", "joint_a6"};
  for (int i = 0; i < 6; ++i) {
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
        kJointNames[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i]));
  }
  registerInterface(&joint_state_interface_);

  CartesianStateHandle state("kuka_tcp", &cart_pos_[0], &cart_pos_[1],
                             &cart_pos_[2], &cart_pos_[3], &cart_pos_[4],
                             &cart_pos_[5]);
  cartesian_state_interface_.registerHandle(state);
  registerInterface(&cartesian_state_interface_);

  correction_command_interface_.registerHandle(CartesianCorrectionHandle(
      state, &cart_cmd_[0], &cart_cmd_[1], &cart_cmd_[2], &cart_cmd_[3],
      &cart_cmd_[4], &cart_cmd_[5]));
  registerInterface(&correction_command_interface_);
}

void KukaRsiRobotHW::read(const ros::Time& /*time*/,
                          const ros::Duration& /*period*/) {
  if (fault_clear_requested_.exchange(false)) monitor_.clearFault();
  const int n = udp_.receive(rx_buf_, sizeof(rx_buf_) - 1, cfg_.read_timeout_ms);
  if (n <= 0) {
    monitor_.onTimeout();
    return;
  }
  rx_buf_[n] = '\0';
  RobFrame frame;
  if (!parseRobFrame(rx_buf_, static_cast<std::size_t>(n), frame)) {
    monitor_.onBadFrame();
    return;
  }
  monitor_.onFrame(frame);
  last_ipoc_ = frame.ipoc;
  cart_pos_[0] = frame.x;
  cart_pos_[1] = frame.y;
  cart_pos_[2] = frame.z;
  cart_pos_[3] = frame.a;
  cart_pos_[4] = frame.b;
  cart_pos_[5] = frame.c;
  for (int i = 0; i < 6; ++i) joint_pos_[i] = frame.axis_deg[i] * kDegToRad;
}

void KukaRsiRobotHW::write(const ros::Time& /*time*/,
                           const ros::Duration& /*period*/) {
  if (!monitor_.connected()) return;  // nobody to answer yet

  SenFrame out;
  out.ipoc = last_ipoc_;
  out.watchdog = ++watchdog_;
  if (monitor_.faulted()) {
    out.stop = 1;  // zero correction + stop request; fault is latched
  } else {
    if (limiter_.clamp(cart_cmd_, cfg_.limits)) ++saturation_count_;
    out.x = cart_cmd_[0];
    out.y = cart_cmd_[1];
    out.z = cart_cmd_[2];
    out.a = cart_cmd_[3];
    out.b = cart_cmd_[4];
    out.c = cart_cmd_[5];
  }
  // Corrections are per-cycle increments: consume-and-clear so a stale
  // command is never re-sent (spec 6.1 stale-command rule).
  for (double& v : cart_cmd_) v = 0.0;

  const std::size_t len = serializeSenFrame(out, tx_buf_, sizeof(tx_buf_));
  if (len == 0) return;
  udp_.sendToLastSender(tx_buf_, len);
}

}  // namespace kuka_rsi
