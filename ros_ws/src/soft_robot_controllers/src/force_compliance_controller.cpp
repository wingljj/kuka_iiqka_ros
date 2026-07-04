#include "soft_robot_controllers/force_compliance_controller.h"

#include <pluginlib/class_list_macros.hpp>

#include <string>

namespace soft_robot_controllers {

namespace {

void loadSafety(ros::NodeHandle& nh, sfc::SafetyParams& s) {
  nh.param("safety/max_corr_trans", s.max_corr_trans, s.max_corr_trans);
  nh.param("safety/max_corr_rot", s.max_corr_rot, s.max_corr_rot);
  nh.param("safety/force_ceiling", s.force_ceiling, s.force_ceiling);
  nh.param("safety/torque_ceiling", s.torque_ceiling, s.torque_ceiling);
}

void loadPayload(ros::NodeHandle& nh, sfc::PayloadParams& p) {
  nh.param("payload/gravity_n", p.gravity_n, p.gravity_n);
  nh.param("payload/com_x", p.com_x, p.com_x);
  nh.param("payload/com_y", p.com_y, p.com_y);
  nh.param("payload/com_z", p.com_z, p.com_z);
  nh.param("payload/bias_fx", p.bias.fx, p.bias.fx);
  nh.param("payload/bias_fy", p.bias.fy, p.bias.fy);
  nh.param("payload/bias_fz", p.bias.fz, p.bias.fz);
  nh.param("payload/bias_tx", p.bias.tx, p.bias.tx);
  nh.param("payload/bias_ty", p.bias.ty, p.bias.ty);
  nh.param("payload/bias_tz", p.bias.tz, p.bias.tz);
}

void loadProfile(ros::NodeHandle& nh, const std::string& name,
                 bool drag_defaults, ForceComplianceParams& p) {
  p.adaptive_deadband = drag_defaults;
  p.retare.enabled = drag_defaults;
  const std::string b = "profiles/" + name + "/";
  nh.param(b + "filter_cutoff_hz", p.filter_cutoff_hz, p.filter_cutoff_hz);
  nh.param(b + "speed_scale", p.compliance.speed_scale,
           p.compliance.speed_scale);
  nh.param(b + "gain_translation", p.compliance.translation.gain,
           p.compliance.translation.gain);
  nh.param(b + "gain_rotation", p.compliance.rotation.gain,
           p.compliance.rotation.gain);
  nh.param(b + "max_speed_translation", p.compliance.translation.max_speed,
           p.compliance.translation.max_speed);
  nh.param(b + "max_speed_rotation", p.compliance.rotation.max_speed,
           p.compliance.rotation.max_speed);
  nh.param(b + "max_accel_translation", p.compliance.translation.max_accel,
           p.compliance.translation.max_accel);
  nh.param(b + "max_accel_rotation", p.compliance.rotation.max_accel,
           p.compliance.rotation.max_accel);
  nh.param(b + "deadband_force", p.fixed_force_deadband_n,
           p.fixed_force_deadband_n);
  nh.param(b + "deadband_torque", p.fixed_torque_deadband_nm,
           p.fixed_torque_deadband_nm);
  nh.param(b + "adaptive_deadband", p.adaptive_deadband, p.adaptive_deadband);
  nh.param(b + "ramp_window_s", p.ramp_window_s, p.ramp_window_s);
  nh.param(b + "ramp_force_margin_n", p.ramp_force_margin_n,
           p.ramp_force_margin_n);
  nh.param(b + "ramp_torque_margin_nm", p.ramp_torque_margin_nm,
           p.ramp_torque_margin_nm);
  nh.param(b + "auto_retare", p.retare.enabled, p.retare.enabled);
  nh.param(b + "retare_orientation_tol_deg", p.retare.orientation_tol_deg,
           p.retare.orientation_tol_deg);
  nh.param(b + "retare_rearm_factor", p.retare_rearm_factor,
           p.retare_rearm_factor);
}

}  // namespace

bool ForceComplianceController::init(
    kuka_rsi::CartesianCorrectionCommandInterface* hw, ros::NodeHandle& root_nh,
    ros::NodeHandle& controller_nh) {
  ForceComplianceParams drag;
  ForceComplianceParams precision;
  loadProfile(controller_nh, "drag", true, drag);
  loadProfile(controller_nh, "precision", false, precision);

  sfc::SafetyParams safety;
  loadSafety(controller_nh, safety);
  sfc::PayloadParams payload;
  loadPayload(controller_nh, payload);
  double wrench_timeout = 0.012;
  controller_nh.param("wrench_timeout", wrench_timeout, wrench_timeout);
  std::vector<double> mount_rpy{0.0, 0.0, 0.0};
  controller_nh.param("sensor_to_flange_rpy", mount_rpy, mount_rpy);
  if (mount_rpy.size() == 3) {
    mount_a_ = mount_rpy[0];
    mount_b_ = mount_rpy[1];
    mount_c_ = mount_rpy[2];
  } else {
    ROS_WARN("sensor_to_flange_rpy must have 3 elements; using identity");
  }
  drag.safety = safety;
  precision.safety = safety;
  drag.payload = payload;
  precision.payload = payload;
  drag.wrench_timeout_s = wrench_timeout;
  precision.wrench_timeout_s = wrench_timeout;

  std::string resource;
  controller_nh.param<std::string>("cartesian_resource", resource,
                                   std::string("kuka_tcp"));
  kuka_rsi::CartesianCorrectionHandle handle;
  try {
    handle = hw->getHandle(resource);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ForceComplianceController: " << ex.what());
    return false;
  }
  if (!configureController(handle, drag, precision)) return false;

  std::string wrench_topic;
  std::string mode_topic;
  std::string rsi_topic;
  std::string state_topic;
  controller_nh.param<std::string>("wrench_topic", wrench_topic,
                                   std::string("/sri_ft/wrench_raw"));
  controller_nh.param<std::string>("mode_command_topic", mode_topic,
                                   std::string("/soft_robot/mode_command"));
  controller_nh.param<std::string>("rsi_state_topic", rsi_topic,
                                   std::string("/kuka/rsi/state"));
  controller_nh.param<std::string>("mode_state_topic", state_topic,
                                   std::string("/soft_robot/mode_state"));
  wrench_sub_ = root_nh.subscribe(wrench_topic, 1,
                                  &ForceComplianceController::wrenchCb, this);
  mode_sub_ = root_nh.subscribe(mode_topic, 1,
                                &ForceComplianceController::modeCb, this);
  rsi_sub_ = root_nh.subscribe(rsi_topic, 1,
                               &ForceComplianceController::rsiStateCb, this);
  std::string eki_topic;
  controller_nh.param<std::string>("eki_state_topic", eki_topic,
                                   std::string("/kuka/eki/state"));
  eki_sub_ =
      root_nh.subscribe(eki_topic, 1, &ForceComplianceController::ekiStateCb,
                        this);
  state_pub_.reset(
      new realtime_tools::RealtimePublisher<soft_robot_msgs::ModeState>(
          root_nh, state_topic, 4));
  return true;
}

bool ForceComplianceController::configureController(
    const kuka_rsi::CartesianCorrectionHandle& handle,
    const ForceComplianceParams& drag, const ForceComplianceParams& precision) {
  handle_ = handle;
  drag_params_ = drag;
  precision_params_ = precision;
  core_.configure(precision_params_);  // safe defaults until a mode entry
  wrench_buf_.writeFromNonRT(WrenchSample{});
  mode_buf_.writeFromNonRT(ModeRequest{});
  fault_buf_.writeFromNonRT(FaultFlag{});
  tool_buf_.writeFromNonRT(ToolSample{});
  return true;
}

void ForceComplianceController::injectModeCommand(std::uint8_t mode,
                                                  std::uint8_t profile) {
  ModeRequest r;
  r.mode = mode;
  r.profile = profile;
  r.seq = ++mode_seq_;
  mode_buf_.writeFromNonRT(r);
}

void ForceComplianceController::wrenchCb(
    const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  WrenchSample s;
  s.w.fx = msg->wrench.force.x;
  s.w.fy = msg->wrench.force.y;
  s.w.fz = msg->wrench.force.z;
  s.w.tx = msg->wrench.torque.x;
  s.w.ty = msg->wrench.torque.y;
  s.w.tz = msg->wrench.torque.z;
  s.stamp_s = msg->header.stamp.isZero() ? ros::Time::now().toSec()
                                         : msg->header.stamp.toSec();
  s.valid = true;
  injectWrench(s);
}

void ForceComplianceController::modeCb(
    const soft_robot_msgs::ModeCommand::ConstPtr& msg) {
  injectModeCommand(msg->mode, msg->profile);
}

void ForceComplianceController::rsiStateCb(
    const soft_robot_msgs::RsiState::ConstPtr& msg) {
  injectRsiFault(msg->fault);
}

void ForceComplianceController::ekiStateCb(
    const soft_robot_msgs::EkiState::ConstPtr& msg) {
  if (!msg->connected) return;  // keep the last known tool
  ToolSample t;
  t.a = msg->tool_a;
  t.b = msg->tool_b;
  t.c = msg->tool_c;
  t.valid = true;
  injectTool(t);
}

sfc::CartesianState ForceComplianceController::readState() const {
  sfc::CartesianState s;
  s.x = handle_.getX();
  s.y = handle_.getY();
  s.z = handle_.getZ();
  s.a = handle_.getA();
  s.b = handle_.getB();
  s.c = handle_.getC();
  return s;
}

void ForceComplianceController::activateCore() {
  const ToolSample tool = *tool_buf_.readFromRT();
  ToolFrameConfig tf;
  if (tool.valid) {
    tf.tool_a = tool.a;
    tf.tool_b = tool.b;
    tf.tool_c = tool.c;
  } else {
    ROS_WARN_ONCE(
        "FORCE_COMPLIANCE activated without EKI tool data; assuming identity "
        "tool rotation");
  }
  tf.mount_a = mount_a_;
  tf.mount_b = mount_b_;
  tf.mount_c = mount_c_;
  if (core_.payload().gravity_n == 0.0) {
    ROS_WARN_ONCE(
        "payload not calibrated (gravity_n == 0): zero-only mode, "
        "orientation changes will drift");
  }
  core_.activate(readState(), tf);
}

void ForceComplianceController::setZero() {
  handle_.setCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void ForceComplianceController::starting(const ros::Time& /*time*/) {
  setZero();
  if (gate_.engaged()) {
    // Controller restarted while the mode is still active: re-activate so
    // the ramp/re-tare reference matches the current pose.
    activateCore();
  }
}

void ForceComplianceController::update(const ros::Time& time,
                                       const ros::Duration& period) {
  const ModeRequest req = *mode_buf_.readFromRT();
  if (gate_.apply(req)) {  // entered FORCE_COMPLIANCE on this cycle
    const bool drag = gate_.snapshot().profile == sfc::Profile::DRAG;
    core_.configure(drag ? drag_params_ : precision_params_);
    activateCore();
  }
  const bool fault = fault_buf_.readFromRT()->fault;
  if (!gate_.engaged() || fault) {
    setZero();
    publishState(time, fault);
    return;
  }

  const WrenchSample ws = *wrench_buf_.readFromRT();
  ComplianceInput in;
  in.state = readState();
  in.raw = ws.w;
  in.wrench_valid = ws.valid;
  in.wrench_age_s = ws.valid ? time.toSec() - ws.stamp_s : 1e9;
  // Plan 2 follow-up 1: dt is the measured period, never a constant.
  const ComplianceOutput out = core_.update(in, period.toSec());

  handle_.setCommand(out.correction.x, out.correction.y, out.correction.z,
                     out.correction.a, out.correction.b, out.correction.c);
  publishState(time, out.hard_cutoff || out.wrench_timeout);
}

void ForceComplianceController::stopping(const ros::Time& /*time*/) {
  setZero();
  gate_.forceIdle();
}

void ForceComplianceController::publishState(const ros::Time& time,
                                             bool degraded) {
  if (!state_pub_) return;  // offline tests run without a publisher
  if (last_pub_s_ >= 0.0 && time.toSec() - last_pub_s_ < 0.02) return;
  if (!state_pub_->trylock()) return;
  last_pub_s_ = time.toSec();
  const sfc::ModeSnapshot snap = gate_.snapshot();
  state_pub_->msg_.header.stamp = time;
  state_pub_->msg_.mode = fromControlMode(snap.mode);
  state_pub_->msg_.profile = fromProfile(snap.profile);
  if (!gate_.engaged()) {
    state_pub_->msg_.system_state = soft_robot_msgs::ModeState::SYSTEM_READY;
  } else if (degraded) {
    state_pub_->msg_.system_state =
        soft_robot_msgs::ModeState::SYSTEM_DEGRADED;
  } else {
    state_pub_->msg_.system_state =
        soft_robot_msgs::ModeState::SYSTEM_SERVOING;
  }
  state_pub_->unlockAndPublish();
}

}  // namespace soft_robot_controllers

PLUGINLIB_EXPORT_CLASS(soft_robot_controllers::ForceComplianceController,
                       controller_interface::ControllerBase)
