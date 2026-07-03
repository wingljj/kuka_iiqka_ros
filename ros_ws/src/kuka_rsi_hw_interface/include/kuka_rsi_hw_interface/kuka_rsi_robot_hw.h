#pragma once

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <atomic>
#include <cstdint>
#include <string>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"
#include "kuka_rsi_hw_interface/command_limiter.h"
#include "kuka_rsi_hw_interface/rsi_frame.h"
#include "kuka_rsi_hw_interface/rsi_session_monitor.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace kuka_rsi {

struct HwConfig {
  std::string listen_ip{"0.0.0.0"};
  std::uint16_t listen_port{49152};
  int read_timeout_ms{8};  // 2 RSI cycles
  unsigned max_consecutive_timeouts{5};
  CommandLimits limits;
};

// ros_control hardware interface for the KUKA RSI channel (spec 5.1, 12.2).
// The KRC is the UDP client: read() waits (bounded) for its state frame,
// write() answers with the RKorr correction echoing the received IPOC.
// Owns no force-control logic. configure() is the ROS-master-free entry
// used by tests and by init().
class KukaRsiRobotHW : public hardware_interface::RobotHW {
 public:
  KukaRsiRobotHW();

  bool configure(const HwConfig& cfg);
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  bool connected() const { return monitor_.connected(); }
  bool faulted() const { return monitor_.faulted(); }
  const SessionStats& sessionStats() const { return monitor_.stats(); }
  std::uint64_t saturationCount() const { return saturation_count_; }
  // Immediate variant (control-thread / test use): keeps cumulative
  // counters (Plan 4 follow-up 10).
  void resetFault() { monitor_.clearFault(); }
  // Thread-safe variant for the node's service callback: applied at the
  // start of the next read() so only the control thread touches monitor_.
  void requestFaultClear() { fault_clear_requested_.store(true); }
  std::uint16_t listenPort() const { return udp_.boundPort(); }

 private:
  void registerInterfaces();

  HwConfig cfg_;
  UdpTransport udp_;
  RsiSessionMonitor monitor_;
  CommandLimiter limiter_;

  // State buffers (written by read(), exposed through interfaces).
  double cart_pos_[6] = {0, 0, 0, 0, 0, 0};      // x y z a b c [mm/deg]
  double joint_pos_[6] = {0, 0, 0, 0, 0, 0};     // rad
  double joint_vel_[6] = {0, 0, 0, 0, 0, 0};     // always 0 (RSI: no velocity)
  double joint_eff_[6] = {0, 0, 0, 0, 0, 0};     // always 0
  // Command buffer (written by controllers, consumed + cleared by write()).
  double cart_cmd_[6] = {0, 0, 0, 0, 0, 0};

  std::uint64_t last_ipoc_{0};
  std::uint64_t watchdog_{0};
  std::uint64_t saturation_count_{0};
  std::atomic<bool> fault_clear_requested_{false};
  char rx_buf_[1024];
  char tx_buf_[1024];

  hardware_interface::JointStateInterface joint_state_interface_;
  CartesianStateInterface cartesian_state_interface_;
  CartesianCorrectionCommandInterface correction_command_interface_;
};

}  // namespace kuka_rsi
