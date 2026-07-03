#include "soft_robot_controllers/cartesian_correction_controller.h"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cstring>
#include <string>

namespace soft_robot_controllers {

namespace {

std::uint64_t packDouble(double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  return bits;
}

double unpackDouble(std::uint64_t bits) {
  double v = 0.0;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

void loadSafety(ros::NodeHandle& nh, sfc::SafetyParams& s) {
  nh.param("safety/max_corr_trans", s.max_corr_trans, s.max_corr_trans);
  nh.param("safety/max_corr_rot", s.max_corr_rot, s.max_corr_rot);
  nh.param("safety/force_ceiling", s.force_ceiling, s.force_ceiling);
  nh.param("safety/torque_ceiling", s.torque_ceiling, s.torque_ceiling);
}

}  // namespace

bool CartesianCorrectionController::init(
    kuka_rsi::CartesianCorrectionCommandInterface* hw, ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh) {
  CartesianCorrectionControllerParams p;
  controller_nh.param("stream_timeout", p.direct.stream_timeout_s,
                      p.direct.stream_timeout_s);
  loadSafety(controller_nh, p.direct.safety);
  controller_nh.param("goal/max_speed_dps", p.goal_defaults.max_speed_dps,
                      p.goal_defaults.max_speed_dps);
  controller_nh.param("goal/p_gain", p.goal_defaults.p_gain,
                      p.goal_defaults.p_gain);
  controller_nh.param("goal/tol_deg", p.goal_defaults.tol_deg,
                      p.goal_defaults.tol_deg);
  controller_nh.param("goal/hold_s", p.goal_defaults.hold_s,
                      p.goal_defaults.hold_s);
  controller_nh.param("goal/timeout_s", p.goal_defaults.timeout_s,
                      p.goal_defaults.timeout_s);

  std::string resource;
  controller_nh.param<std::string>("cartesian_resource", resource,
                                   std::string("kuka_tcp"));
  kuka_rsi::CartesianCorrectionHandle handle;
  try {
    handle = hw->getHandle(resource);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("CartesianCorrectionController: " << ex.what());
    return false;
  }
  if (!configureController(handle, p)) return false;

  std::string command_topic;
  std::string mode_topic;
  std::string rsi_topic;
  std::string action_name;
  controller_nh.param<std::string>(
      "command_topic", command_topic,
      std::string("/soft_robot/cartesian_correction_command"));
  controller_nh.param<std::string>("mode_command_topic", mode_topic,
                                   std::string("/soft_robot/mode_command"));
  controller_nh.param<std::string>("rsi_state_topic", rsi_topic,
                                   std::string("/kuka/rsi/state"));
  controller_nh.param<std::string>(
      "action_name", action_name,
      std::string("/soft_robot/move_to_orientation"));
  stream_sub_ = root_nh.subscribe(
      command_topic, 1, &CartesianCorrectionController::streamCb, this);
  mode_sub_ = root_nh.subscribe(mode_topic, 1,
                                &CartesianCorrectionController::modeCb, this);
  rsi_sub_ = root_nh.subscribe(
      rsi_topic, 1, &CartesianCorrectionController::rsiStateCb, this);
  action_server_.reset(new ActionServer(
      root_nh, action_name,
      [this](const soft_robot_msgs::MoveToOrientationGoalConstPtr& g) {
        executeCb(g);
      },
      false));
  action_server_->start();
  return true;
}

bool CartesianCorrectionController::configureController(
    const kuka_rsi::CartesianCorrectionHandle& handle,
    const CartesianCorrectionControllerParams& params) {
  handle_ = handle;
  params_ = params;
  core_.configure(params_.direct);
  stream_buf_.writeFromNonRT(StreamCommand{});
  mode_buf_.writeFromNonRT(ModeRequest{});
  fault_buf_.writeFromNonRT(FaultFlag{});
  goal_buf_.writeFromNonRT(GoalRequest{});
  status_.store(static_cast<int>(sfc::MotionStatus::INACTIVE));
  applied_seq_pub_.store(0);
  error_bits_.store(packDouble(0.0));
  engaged_flag_.store(false);
  return true;
}

void CartesianCorrectionController::injectModeCommand(std::uint8_t mode,
                                                      std::uint8_t profile) {
  ModeRequest r;
  r.mode = mode;
  r.profile = profile;
  r.seq = ++mode_seq_;
  mode_buf_.writeFromNonRT(r);
}

std::uint64_t CartesianCorrectionController::requestGoal(
    const sfc::MotionGoal& g) {
  GoalRequest r;
  r.goal = g;
  r.cancel = false;
  r.seq = ++goal_seq_;
  goal_buf_.writeFromNonRT(r);
  return r.seq;
}

std::uint64_t CartesianCorrectionController::requestCancel() {
  GoalRequest r;
  r.cancel = true;
  r.seq = ++goal_seq_;
  goal_buf_.writeFromNonRT(r);
  return r.seq;
}

double CartesianCorrectionController::motionErrorDeg() const {
  return unpackDouble(error_bits_.load());
}

void CartesianCorrectionController::streamCb(
    const soft_robot_msgs::CartesianCorrectionStamped::ConstPtr& msg) {
  StreamCommand c;
  c.correction.x = msg->correction.x;
  c.correction.y = msg->correction.y;
  c.correction.z = msg->correction.z;
  c.correction.a = msg->correction.a;
  c.correction.b = msg->correction.b;
  c.correction.c = msg->correction.c;
  c.stamp_s = msg->header.stamp.isZero() ? ros::Time::now().toSec()
                                         : msg->header.stamp.toSec();
  c.valid = true;
  injectStream(c);
}

void CartesianCorrectionController::modeCb(
    const soft_robot_msgs::ModeCommand::ConstPtr& msg) {
  injectModeCommand(msg->mode, msg->profile);
}

void CartesianCorrectionController::rsiStateCb(
    const soft_robot_msgs::RsiState::ConstPtr& msg) {
  injectRsiFault(msg->fault);
}

sfc::CartesianState CartesianCorrectionController::readState() const {
  sfc::CartesianState s;
  s.x = handle_.getX();
  s.y = handle_.getY();
  s.z = handle_.getZ();
  s.a = handle_.getA();
  s.b = handle_.getB();
  s.c = handle_.getC();
  return s;
}

void CartesianCorrectionController::setZero() {
  handle_.setCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void CartesianCorrectionController::starting(const ros::Time& /*time*/) {
  setZero();
  core_.reset();
}

void CartesianCorrectionController::update(const ros::Time& time,
                                           const ros::Duration& period) {
  const ModeRequest req = *mode_buf_.readFromRT();
  if (gate_.apply(req)) core_.reset();  // entered: drop stale stream/goal
  const bool fault = fault_buf_.readFromRT()->fault;
  const bool engaged = gate_.engaged() && !fault;
  engaged_flag_.store(engaged);

  const GoalRequest gr = *goal_buf_.readFromRT();
  if (gr.seq != 0 && gr.seq != applied_goal_seq_) {
    applied_goal_seq_ = gr.seq;
    if (gr.cancel) {
      core_.cancelGoal();
    } else {
      core_.startGoal(gr.goal);
    }
  }

  const sfc::CartesianState state = readState();
  if (!engaged) {
    // Disengaging cancels a running goal. The action layer maps the
    // resulting INACTIVE to ABORTED unless it cancelled itself
    // (Plan 1 follow-up 5 bookkeeping, decision 5).
    if (core_.goalStatus() == sfc::MotionStatus::RUNNING) core_.cancelGoal();
    setZero();
  } else {
    core_.setStream(*stream_buf_.readFromRT());
    // Plan 2 follow-up 1: dt is the measured period, never a constant.
    const DirectOutput out =
        core_.update(state, time.toSec(), period.toSec());
    handle_.setCommand(out.correction.x, out.correction.y, out.correction.z,
                       out.correction.a, out.correction.b, out.correction.c);
  }
  status_.store(static_cast<int>(core_.goalStatus()));
  error_bits_.store(packDouble(core_.goalErrorDeg(state)));
  applied_seq_pub_.store(applied_goal_seq_);
}

void CartesianCorrectionController::stopping(const ros::Time& /*time*/) {
  setZero();
  gate_.forceIdle();
  requestCancel();  // a goal must not survive a controller stop
}

void CartesianCorrectionController::executeCb(
    const soft_robot_msgs::MoveToOrientationGoalConstPtr& goal) {
  soft_robot_msgs::MoveToOrientationResult result;
  if (goal->use_position) {
    result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
    result.message =
        "position targets are not supported in v1 (orientation only)";
    action_server_->setAborted(result);
    return;
  }
  if (!engagedNow()) {
    result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
    result.message = "controller is not engaged (mode gate)";
    action_server_->setAborted(result);
    return;
  }
  sfc::MotionGoal g = params_.goal_defaults;
  g.a = goal->a;
  g.b = goal->b;
  g.c = goal->c;
  g.max_speed_dps =
      params_.goal_defaults.max_speed_dps * clampSpeedScale(goal->speed_scale);
  const std::uint64_t seq = requestGoal(g);

  const ros::Time deadline =
      ros::Time::now() + ros::Duration(g.timeout_s + 5.0);
  bool cancelled = false;
  double initial_error = -1.0;
  ros::Rate rate(50);
  while (ros::ok()) {
    if (ros::Time::now() > deadline) {
      requestCancel();
      result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
      result.message = "realtime loop did not process the goal in time";
      action_server_->setAborted(result);
      return;
    }
    if (!cancelled && action_server_->isPreemptRequested()) {
      requestCancel();
      cancelled = true;
    }
    if (appliedGoalSeq() >= seq) {
      const sfc::MotionStatus st = motionStatus();
      if (st == sfc::MotionStatus::CONVERGED) {
        result.result_code =
            soft_robot_msgs::MoveToOrientationResult::CONVERGED;
        result.message = "converged";
        action_server_->setSucceeded(result);
        return;
      }
      if (st == sfc::MotionStatus::TIMEOUT) {
        result.result_code = soft_robot_msgs::MoveToOrientationResult::TIMEOUT;
        result.message = "motion timed out before convergence";
        action_server_->setAborted(result);
        return;
      }
      if (st == sfc::MotionStatus::INACTIVE) {
        // INACTIVE cannot distinguish our own cancel from a realtime-side
        // cancel (Plan 1 follow-up 5): the shell keeps the bookkeeping.
        result.result_code = soft_robot_msgs::MoveToOrientationResult::ABORTED;
        if (cancelled) {
          result.message = "preempted";
          action_server_->setPreempted(result);
        } else {
          result.message = "goal cancelled by mode change or fault";
          action_server_->setAborted(result);
        }
        return;
      }
      const double err = motionErrorDeg();
      if (initial_error < 0.0 && err > 0.0) initial_error = err;
      soft_robot_msgs::MoveToOrientationFeedback fb;
      fb.error_deg = err;
      fb.error_mm = 0.0;
      fb.progress =
          initial_error > 0.0
              ? std::max(0.0, std::min(1.0, 1.0 - err / initial_error))
              : 0.0;
      action_server_->publishFeedback(fb);
    }
    rate.sleep();
  }
  requestCancel();  // node shutdown while a goal was active
}

}  // namespace soft_robot_controllers

PLUGINLIB_EXPORT_CLASS(soft_robot_controllers::CartesianCorrectionController,
                       controller_interface::ControllerBase)
