#pragma once

#include <actionlib/server/simple_action_server.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <soft_robot_msgs/CartesianCorrectionStamped.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>

#include <atomic>
#include <cstdint>
#include <memory>

#include "kuka_rsi_hw_interface/cartesian_command_interface.h"
#include "soft_robot_controllers/controller_mode_gate.h"
#include "soft_robot_controllers/direct_correction_core.h"

namespace soft_robot_controllers {

struct CartesianCorrectionControllerParams {
  DirectCorrectionParams direct;
  // Template for action goals: a/b/c are overwritten per goal and
  // max_speed_dps is scaled by the goal's speed_scale (decision 8).
  sfc::MotionGoal goal_defaults;
};

// Goal/cancel request handed from the action thread to update() through a
// RealtimeBuffer. seq == 0 means "no request yet".
struct GoalRequest {
  sfc::MotionGoal goal;
  bool cancel{false};
  std::uint64_t seq{0};
};

// Direct Cartesian correction controller (spec 5.3, 7.6, 7.7): stream
// passthrough for commissioning/jog, plus the RKorr goal mode hosting
// OrientationMotionCore behind /soft_robot/move_to_orientation. Engaged
// in DIRECT_CARTESIAN and CALIBRATION modes. The action execute thread
// never touches realtime data structures: goals travel through a
// RealtimeBuffer, status travels back through atomics (decision 5).
class CartesianCorrectionController
    : public controller_interface::Controller<
          kuka_rsi::CartesianCorrectionCommandInterface> {
 public:
  bool init(kuka_rsi::CartesianCorrectionCommandInterface* hw,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // ROS-master-free wiring entry used by init() and by offline tests.
  bool configureController(const kuka_rsi::CartesianCorrectionHandle& handle,
                           const CartesianCorrectionControllerParams& params);

  // Non-RT producers (subscriber callbacks and tests).
  void injectStream(const StreamCommand& cmd) {
    stream_buf_.writeFromNonRT(cmd);
  }
  void injectModeCommand(std::uint8_t mode, std::uint8_t profile);
  void injectRsiFault(bool fault) {
    fault_buf_.writeFromNonRT(FaultFlag{fault});
  }

  // Goal plumbing shared by the action execute thread and offline tests.
  std::uint64_t requestGoal(const sfc::MotionGoal& g);
  std::uint64_t requestCancel();
  std::uint64_t appliedGoalSeq() const { return applied_seq_pub_.load(); }
  sfc::MotionStatus motionStatus() const {
    return static_cast<sfc::MotionStatus>(status_.load());
  }
  double motionErrorDeg() const;
  bool engagedNow() const { return engaged_flag_.load(); }

  static double clampSpeedScale(double scale) {
    return (scale <= 0.0 || scale > 1.0) ? 1.0 : scale;
  }

 private:
  using ActionServer =
      actionlib::SimpleActionServer<soft_robot_msgs::MoveToOrientationAction>;

  void streamCb(
      const soft_robot_msgs::CartesianCorrectionStamped::ConstPtr& msg);
  void modeCb(const soft_robot_msgs::ModeCommand::ConstPtr& msg);
  void rsiStateCb(const soft_robot_msgs::RsiState::ConstPtr& msg);
  void executeCb(const soft_robot_msgs::MoveToOrientationGoalConstPtr& goal);
  sfc::CartesianState readState() const;
  void setZero();

  kuka_rsi::CartesianCorrectionHandle handle_;
  ControllerModeGate gate_{sfc::ControlMode::DIRECT_CARTESIAN,
                           sfc::ControlMode::CALIBRATION};
  DirectCorrectionCore core_;
  CartesianCorrectionControllerParams params_;

  realtime_tools::RealtimeBuffer<StreamCommand> stream_buf_;
  realtime_tools::RealtimeBuffer<ModeRequest> mode_buf_;
  realtime_tools::RealtimeBuffer<FaultFlag> fault_buf_;
  realtime_tools::RealtimeBuffer<GoalRequest> goal_buf_;
  std::uint64_t mode_seq_{0};                  // producer side
  std::atomic<std::uint64_t> goal_seq_{0};     // producer side (AS thread)
  std::uint64_t applied_goal_seq_{0};          // RT side only
  std::atomic<std::uint64_t> applied_seq_pub_{0};   // RT -> AS
  std::atomic<int> status_{static_cast<int>(sfc::MotionStatus::INACTIVE)};
  std::atomic<std::uint64_t> error_bits_{0};        // packed double
  std::atomic<bool> engaged_flag_{false};

  ros::Subscriber stream_sub_;
  ros::Subscriber mode_sub_;
  ros::Subscriber rsi_sub_;
  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace soft_robot_controllers
