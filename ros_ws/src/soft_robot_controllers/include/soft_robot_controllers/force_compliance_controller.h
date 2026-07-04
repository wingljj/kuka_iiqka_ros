#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/RsiState.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"
#include "soft_robot_controllers/controller_mode_gate.h"
#include "soft_robot_controllers/force_compliance_core.h"

namespace soft_robot_controllers {

// Latest wrench sample handed from the subscriber thread to update().
struct WrenchSample {
  sfc::Wrench w;
  double stamp_s{0};
  bool valid{false};
};

// Latest $TOOL A/B/C from the EKI bridge; locked into the core at mode
// entry (session-constant, spec tool-frame design). valid stays false
// until the first connected EkiState arrives.
struct ToolSample {
  double a{0}, b{0}, c{0};
  bool valid{false};
};

// Thin ROS shell around ForceComplianceCore (spec 5.3 / 7.1). All topic
// input reaches update() through RealtimeBuffers written by subscriber
// callbacks on the node spinner threads; update() only reads buffers and
// never subscribes, allocates, or blocks. Fault awareness comes from the
// /kuka/rsi/state topic (Plan 2 follow-up 3, decision 2).
class ForceComplianceController
    : public controller_interface::Controller<
          kuka_rsi::CartesianCorrectionCommandInterface> {
 public:
  bool init(kuka_rsi::CartesianCorrectionCommandInterface* hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // ROS-master-free wiring entry (same pattern as KukaRsiRobotHW::
  // configure): used by init() and directly by offline tests.
  bool configureController(const kuka_rsi::CartesianCorrectionHandle& handle,
                           const ForceComplianceParams& drag,
                           const ForceComplianceParams& precision);

  // Non-RT producers. Subscriber callbacks delegate here; offline tests
  // call them directly (single producer per buffer).
  void injectWrench(const WrenchSample& s) { wrench_buf_.writeFromNonRT(s); }
  void injectModeCommand(std::uint8_t mode, std::uint8_t profile);
  void injectRsiFault(bool fault) {
    fault_buf_.writeFromNonRT(FaultFlag{fault});
  }
  void injectTool(const ToolSample& t) { tool_buf_.writeFromNonRT(t); }

  const ControllerModeGate& gate() const { return gate_; }

 private:
  void wrenchCb(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void modeCb(const soft_robot_msgs::ModeCommand::ConstPtr& msg);
  void rsiStateCb(const soft_robot_msgs::RsiState::ConstPtr& msg);
  void ekiStateCb(const soft_robot_msgs::EkiState::ConstPtr& msg);
  sfc::CartesianState readState() const;
  void activateCore();
  void setZero();
  void publishState(const ros::Time& time, bool degraded);

  kuka_rsi::CartesianCorrectionHandle handle_;
  ControllerModeGate gate_{sfc::ControlMode::FORCE_COMPLIANCE};
  ForceComplianceCore core_;
  ForceComplianceParams drag_params_;
  ForceComplianceParams precision_params_;

  realtime_tools::RealtimeBuffer<WrenchSample> wrench_buf_;
  realtime_tools::RealtimeBuffer<ModeRequest> mode_buf_;
  realtime_tools::RealtimeBuffer<FaultFlag> fault_buf_;
  realtime_tools::RealtimeBuffer<ToolSample> tool_buf_;
  std::uint64_t mode_seq_{0};  // producer side (subscriber thread / tests)
  double mount_a_{0}, mount_b_{0}, mount_c_{0};  // sensor_to_flange_rpy [deg]

  ros::Subscriber wrench_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber rsi_sub_;
  ros::Subscriber eki_sub_;
  std::unique_ptr<
      realtime_tools::RealtimePublisher<soft_robot_msgs::ModeState>>
      state_pub_;
  double last_pub_s_{-1.0};
};

}  // namespace soft_robot_controllers
