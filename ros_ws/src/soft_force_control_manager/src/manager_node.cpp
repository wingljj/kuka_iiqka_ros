// Thin ROS shell around ManagerRuntime (spec 5.5, 9, 11). All orchestration
// logic lives in the offline-tested library; this file loads parameters,
// wires ManagerOps to the real ROS clients, and forwards services/actions.
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <soft_robot_msgs/CalibratePayloadAction.h>
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/ManagerState.h>
#include <soft_robot_msgs/ModeCommand.h>
#include <soft_robot_msgs/ModeState.h>
#include <soft_robot_msgs/MoveToOrientationAction.h>
#include <soft_robot_msgs/RsiState.h>
#include <soft_robot_msgs/SriStatus.h>
#include <soft_robot_msgs/StartServo.h>
#include <std_srvs/Trigger.h>

#include <cstdio>
#include <ctime>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "soft_force_control_manager/manager_runtime.h"

namespace {

// Wire/state numbering alignment (Plan 3 mode_bridge.h pattern).
static_assert(soft_robot_msgs::ModeState::SYSTEM_OFFLINE ==
                  static_cast<std::uint8_t>(sfm::SystemState::OFFLINE),
              "SYSTEM_OFFLINE mismatch");
static_assert(soft_robot_msgs::ModeState::SYSTEM_SERVOING ==
                  static_cast<std::uint8_t>(sfm::SystemState::SERVOING),
              "SYSTEM_SERVOING mismatch");
static_assert(soft_robot_msgs::ModeState::SYSTEM_FAULT ==
                  static_cast<std::uint8_t>(sfm::SystemState::FAULT),
              "SYSTEM_FAULT mismatch");
static_assert(soft_robot_msgs::ModeCommand::MODE_CALIBRATION == 3u,
              "MODE_CALIBRATION wire value drifted");

bool callTrigger(ros::ServiceClient& c) {
  std_srvs::Trigger srv;
  return c.call(srv) && srv.response.success;
}

const char* phaseName(sfm::CalPhase p) {
  switch (p) {
    case sfm::CalPhase::MOVING: return "MOVING";
    case sfm::CalPhase::SETTLING: return "SETTLING";
    case sfm::CalPhase::SAMPLING: return "SAMPLING";
    case sfm::CalPhase::RETURNING: return "RETURNING";
    case sfm::CalPhase::DONE: return "DONE";
    case sfm::CalPhase::FAILED: return "FAILED";
    default: return "IDLE";
  }
}

std::string isoNow() {
  char buf[32];
  const std::time_t t = static_cast<std::time_t>(ros::Time::now().sec);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_utc);
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "soft_robot_manager");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sfm::ManagerConfig cfg;
  pnh.param("tick_period_s", cfg.tick_period_s, 0.1);
  pnh.param("eki_state_timeout_s", cfg.eki_state_timeout_s, 5.0);
  pnh.param("rsi_state_timeout_s", cfg.rsi_state_timeout_s, 0.5);
  pnh.param("sri_status_timeout_s", cfg.sri_status_timeout_s, 2.0);
  pnh.param("tool_sync_retry_s", cfg.tool_sync_retry_s, 2.0);
  pnh.param("rsi_connect_wait_s", cfg.rsi_connect_wait_s, 5.0);
  pnh.param<std::string>("compliance_controller", cfg.compliance_controller,
                         "force_compliance_controller");
  pnh.param<std::string>("correction_controller", cfg.correction_controller,
                         "cartesian_correction_controller");
  double goal_speed_scale = 1.0;
  pnh.param("goal_speed_scale", goal_speed_scale, 1.0);
  std::string payload_file;
  pnh.param<std::string>("payload_file", payload_file, "");
  if (payload_file.empty()) {
    payload_file =
        ros::package::getPath("soft_robot_bringup") + "/config/payload.yaml";
  }

  // calibration.yaml (spec 14): poses / settle / samples / return pose.
  pnh.param("calibration/settle_time_s", cfg.calibration.settle_time_s, 1.0);
  pnh.param("calibration/samples_per_pose", cfg.calibration.samples_per_pose,
            100);
  pnh.param("calibration/return_pose/a", cfg.calibration.return_pose.a, 0.0);
  pnh.param("calibration/return_pose/b", cfg.calibration.return_pose.b, 0.0);
  pnh.param("calibration/return_pose/c", cfg.calibration.return_pose.c, 0.0);
  XmlRpc::XmlRpcValue poses;
  if (pnh.getParam("calibration/poses", poses) &&
      poses.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < poses.size(); ++i) {
      sfm::CalPose p;
      p.a = static_cast<double>(poses[i]["a"]);
      p.b = static_cast<double>(poses[i]["b"]);
      p.c = static_cast<double>(poses[i]["c"]);
      cfg.calibration.poses.push_back(p);
    }
  }
  if (cfg.calibration.poses.empty()) {
    ROS_FATAL("soft_robot_manager: calibration/poses missing or empty");
    return 1;
  }

  // --- ROS clients backing ManagerOps ---
  ros::ServiceClient eki_start =
      nh.serviceClient<std_srvs::Trigger>("/kuka/eki/start_rsi_program");
  ros::ServiceClient eki_stop =
      nh.serviceClient<std_srvs::Trigger>("/kuka/eki/stop_rsi_program");
  ros::ServiceClient eki_reset =
      nh.serviceClient<std_srvs::Trigger>("/kuka/eki/reset_fault");
  ros::ServiceClient eki_get_tool =
      nh.serviceClient<soft_robot_msgs::GetTool>("/kuka/eki/get_tool");
  ros::ServiceClient rsi_reset =
      nh.serviceClient<std_srvs::Trigger>("/kuka/rsi/reset_fault");
  ros::ServiceClient sri_zero =
      nh.serviceClient<std_srvs::Trigger>("/sri_ft/zero");
  ros::ServiceClient cm_switch =
      nh.serviceClient<controller_manager_msgs::SwitchController>(
          "/controller_manager/switch_controller");
  ros::ServiceClient cm_list =
      nh.serviceClient<controller_manager_msgs::ListControllers>(
          "/controller_manager/list_controllers");
  ros::ServiceClient cm_load =
      nh.serviceClient<controller_manager_msgs::LoadController>(
          "/controller_manager/load_controller");
  ros::ServiceClient cm_unload =
      nh.serviceClient<controller_manager_msgs::UnloadController>(
          "/controller_manager/unload_controller");
  ros::Publisher mode_pub =
      nh.advertise<soft_robot_msgs::ModeCommand>("/soft_robot/mode_command", 10);
  ros::Publisher state_pub = nh.advertise<soft_robot_msgs::ManagerState>(
      "/soft_robot/manager_state", 10);
  ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>(
      "/soft_robot/diagnostics", 10);

  actionlib::SimpleActionClient<soft_robot_msgs::MoveToOrientationAction>
      motion_client("/soft_robot/move_to_orientation", true);

  std::unique_ptr<sfm::ManagerRuntime> runtime;

  sfm::ManagerOps ops;
  ops.ekiStartRsi = [&] { return callTrigger(eki_start); };
  ops.ekiStopRsi = [&] { return callTrigger(eki_stop); };
  ops.ekiResetFault = [&] { return callTrigger(eki_reset); };
  ops.rsiResetFault = [&] { return callTrigger(rsi_reset); };
  ops.sriZero = [&] { return callTrigger(sri_zero); };
  ops.ekiGetTool = [&]() -> sfm::ToolFrame {
    sfm::ToolFrame t;
    soft_robot_msgs::GetTool srv;
    if (!eki_get_tool.call(srv) || !srv.response.success) return t;
    t.x = srv.response.x; t.y = srv.response.y; t.z = srv.response.z;
    t.a = srv.response.a; t.b = srv.response.b; t.c = srv.response.c;
    t.valid = true;
    return t;
  };
  // STRICT is safe here: the runtime pre-filters no-op entries against
  // listRunningControllers below (Task 8b), so the request only carries
  // real transitions.
  ops.switchControllers = [&](const std::string& start,
                              const std::string& stop) {
    controller_manager_msgs::SwitchController srv;
    if (!start.empty()) srv.request.start_controllers.push_back(start);
    if (!stop.empty()) srv.request.stop_controllers.push_back(stop);
    srv.request.strictness =
        controller_manager_msgs::SwitchController::Request::STRICT;
    return cm_switch.call(srv) && srv.response.ok;
  };
  ops.listRunningControllers = [&](std::vector<std::string>& running) {
    controller_manager_msgs::ListControllers srv;
    if (!cm_list.call(srv)) return false;
    for (const auto& c : srv.response.controller)
      if (c.state == "running") running.push_back(c.name);
    return true;
  };
  ops.publishMode = [&](std::uint8_t mode, std::uint8_t profile) {
    soft_robot_msgs::ModeCommand msg;
    msg.mode = mode;
    msg.profile = profile;
    mode_pub.publish(msg);
  };
  ops.sendMotionGoal = [&](const sfm::CalPose& p) {
    soft_robot_msgs::MoveToOrientationGoal goal;
    goal.a = p.a; goal.b = p.b; goal.c = p.c;
    goal.use_position = false;
    goal.speed_scale = goal_speed_scale;
    motion_client.sendGoal(
        goal,
        [&](const actionlib::SimpleClientGoalState& state,
            const soft_robot_msgs::MoveToOrientationResultConstPtr&) {
          // SUCCEEDED counts as success even after a preempt raced a
          // convergence (Plan 3 follow-up 7, decision 15).
          runtime->onMotionResult(
              state == actionlib::SimpleClientGoalState::SUCCEEDED);
        });
  };
  ops.applyPayload = [&](const std::string& /*yaml_from_runtime*/,
                         const sfc::PayloadFitResult& fit) {
    // Re-emit with a real timestamp (the runtime has no ROS clock).
    const std::string yaml = sfm::emitPayloadYaml(fit, isoNow());
    std::ofstream f(payload_file, std::ios::trunc);
    if (!f) {
      ROS_ERROR("payload.yaml write failed: %s", payload_file.c_str());
      return false;
    }
    f << yaml;
    f.close();
    // Decision 7: parameter override + controller reload (it is stopped
    // during calibration; ros_control reads parameters at load time).
    const std::string ns = "/" + cfg.compliance_controller + "/payload/";
    ros::param::set(ns + "gravity_n", fit.params.gravity_n);
    ros::param::set(ns + "com_x", fit.params.com_x);
    ros::param::set(ns + "com_y", fit.params.com_y);
    ros::param::set(ns + "com_z", fit.params.com_z);
    ros::param::set(ns + "bias_fx", fit.params.bias.fx);
    ros::param::set(ns + "bias_fy", fit.params.bias.fy);
    ros::param::set(ns + "bias_fz", fit.params.bias.fz);
    ros::param::set(ns + "bias_tx", fit.params.bias.tx);
    ros::param::set(ns + "bias_ty", fit.params.bias.ty);
    ros::param::set(ns + "bias_tz", fit.params.bias.tz);
    controller_manager_msgs::UnloadController u;
    u.request.name = cfg.compliance_controller;
    controller_manager_msgs::LoadController l;
    l.request.name = cfg.compliance_controller;
    if (!cm_unload.call(u) || !u.response.ok ||
        !cm_load.call(l) || !l.response.ok) {
      ROS_ERROR("payload reload of %s failed; new payload takes effect "
                "after restart", cfg.compliance_controller.c_str());
      return false;
    }
    return true;
  };
  ops.publishState = [&] {
    const sfm::ManagerSnapshot s = runtime->snapshot();
    soft_robot_msgs::ManagerState msg;
    msg.header.stamp = ros::Time::now();
    msg.system_state = static_cast<std::uint8_t>(s.state);
    msg.mode = s.mode;
    msg.profile = s.profile;
    msg.eki_connected = s.health.eki_link;
    msg.eki_program_ready = s.health.eki_program_ready;
    msg.rsi_connected = s.health.rsi_connected;
    msg.rsi_fault = s.health.rsi_fault;
    msg.sri_streaming = s.health.sri_streaming;
    msg.tool_synced = s.health.tool_synced;
    msg.calibrating = s.calibrating;
    msg.active_controller = s.active_controller;
    state_pub.publish(msg);
  };

  runtime.reset(new sfm::ManagerRuntime(cfg, ops));

  // --- inbound topics -> runtime feeds ---
  ros::Subscriber eki_sub = nh.subscribe<soft_robot_msgs::EkiState>(
      "/kuka/eki/state", 5, [&](const soft_robot_msgs::EkiState::ConstPtr& m) {
        sfm::EkiFeed f;
        f.connected = m->connected;
        f.state_fresh = m->state_fresh;
        f.program_ready = m->program_ready;
        f.rsi_active = m->rsi_active;
        f.fault = m->fault;
        runtime->feedEkiState(f);
      });
  ros::Subscriber rsi_sub = nh.subscribe<soft_robot_msgs::RsiState>(
      "/kuka/rsi/state", 5, [&](const soft_robot_msgs::RsiState::ConstPtr& m) {
        runtime->feedRsiState(m->connected, m->fault);
      });
  ros::Subscriber sri_sub = nh.subscribe<soft_robot_msgs::SriStatus>(
      "/sri_ft/status", 5, [&](const soft_robot_msgs::SriStatus::ConstPtr& m) {
        runtime->feedSriStatus(m->connected && m->streaming);
      });
  ros::Subscriber wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>(
      "/sri_ft/wrench_raw", 50,
      [&](const geometry_msgs::WrenchStamped::ConstPtr& m) {
        sfc::Wrench w;
        w.fx = m->wrench.force.x; w.fy = m->wrench.force.y;
        w.fz = m->wrench.force.z; w.tx = m->wrench.torque.x;
        w.ty = m->wrench.torque.y; w.tz = m->wrench.torque.z;
        runtime->feedWrench(w);
      });

  // --- controller preloading (READY precondition controllers_loaded) ---
  auto loadController = [&](const std::string& name) {
    controller_manager_msgs::LoadController srv;
    srv.request.name = name;
    return cm_load.call(srv) && srv.response.ok;
  };

  // --- services ---
  ros::ServiceServer start_srv =
      nh.advertiseService<soft_robot_msgs::StartServo::Request,
                          soft_robot_msgs::StartServo::Response>(
          "/soft_robot/start_servo",
          [&](soft_robot_msgs::StartServo::Request& req,
              soft_robot_msgs::StartServo::Response& res) {
            const sfm::CommandResult r =
                runtime->startServo(req.mode, req.profile);
            res.success = r.success;
            res.message = r.message;
            return true;
          });
  auto trigger = [&](sfm::CommandResult (sfm::ManagerRuntime::*fn)()) {
    return [&, fn](std_srvs::Trigger::Request&,
                   std_srvs::Trigger::Response& res) {
      const sfm::CommandResult r = ((*runtime).*fn)();
      res.success = r.success;
      res.message = r.message;
      return true;
    };
  };
  ros::ServiceServer stop_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/soft_robot/stop_servo", trigger(&sfm::ManagerRuntime::stopServo));
  ros::ServiceServer reset_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/soft_robot/reset_fault",
          trigger(&sfm::ManagerRuntime::resetFault));
  ros::ServiceServer zero_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/soft_robot/zero_sensor",
          trigger(&sfm::ManagerRuntime::zeroSensor));

  // --- calibration action server ---
  using CalServer =
      actionlib::SimpleActionServer<soft_robot_msgs::CalibratePayloadAction>;
  std::unique_ptr<CalServer> cal_server;
  cal_server.reset(new CalServer(
      nh, "/soft_robot/calibrate_payload",
      [&](const soft_robot_msgs::CalibratePayloadGoalConstPtr&) {
        soft_robot_msgs::CalibratePayloadResult result;
        const sfm::CommandResult begin = runtime->beginCalibration();
        if (!begin.success) {
          result.success = false;
          result.message = begin.message;
          cal_server->setAborted(result);
          return;
        }
        ros::Rate rate(10);
        while (ros::ok()) {
          if (cal_server->isPreemptRequested()) runtime->cancelCalibration();
          const sfm::ManagerSnapshot s = runtime->snapshot();
          soft_robot_msgs::CalibratePayloadFeedback fb;
          fb.pose_index = static_cast<std::uint32_t>(s.cal.pose_index);
          fb.pose_count = static_cast<std::uint32_t>(s.cal.pose_count);
          fb.phase = phaseName(s.cal.phase);
          cal_server->publishFeedback(fb);
          if (s.cal.phase == sfm::CalPhase::DONE ||
              s.cal.phase == sfm::CalPhase::FAILED) {
            // Wait for the runtime teardown tick to restore READY.
            if (!s.calibrating && s.state != sfm::SystemState::CALIBRATING) {
              result.success =
                  s.cal.phase == sfm::CalPhase::DONE && s.fit.ok;
              result.message =
                  result.success ? (s.cal.return_move_ok
                                        ? "ok"
                                        : "ok (return move failed)")
                                 : std::string("failed in phase ") +
                                       phaseName(s.cal.phase);
              result.gravity_n = s.fit.params.gravity_n;
              result.com_x = s.fit.params.com_x;
              result.com_y = s.fit.params.com_y;
              result.com_z = s.fit.params.com_z;
              result.bias_fx = s.fit.params.bias.fx;
              result.bias_fy = s.fit.params.bias.fy;
              result.bias_fz = s.fit.params.bias.fz;
              result.bias_tx = s.fit.params.bias.tx;
              result.bias_ty = s.fit.params.bias.ty;
              result.bias_tz = s.fit.params.bias.tz;
              result.r2_force = s.fit.r2_force;
              result.r2_torque = s.fit.r2_torque;
              if (result.success) cal_server->setSucceeded(result);
              else cal_server->setAborted(result);
              return;
            }
          }
          rate.sleep();
        }
      },
      false));

  // --- diagnostics (1 Hz) ---
  ros::Timer diag_timer =
      nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        const sfm::ManagerSnapshot s = runtime->snapshot();
        diagnostic_msgs::DiagnosticArray arr;
        arr.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus st;
        st.name = "soft_robot_manager: system";
        st.hardware_id = "soft_robot";
        if (s.state == sfm::SystemState::FAULT) {
          st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
          st.message = "FAULT latched: reset required";
        } else if (s.state == sfm::SystemState::DEGRADED) {
          st.level = diagnostic_msgs::DiagnosticStatus::WARN;
          st.message = "DEGRADED: output forced to zero";
        } else {
          st.level = diagnostic_msgs::DiagnosticStatus::OK;
          st.message = "ok";
        }
        auto kv = [&st](const std::string& k, const std::string& v) {
          diagnostic_msgs::KeyValue e;
          e.key = k;
          e.value = v;
          st.values.push_back(e);
        };
        kv("system_state", std::to_string(static_cast<int>(s.state)));
        kv("eki_link", s.health.eki_link ? "true" : "false");
        kv("rsi_connected", s.health.rsi_connected ? "true" : "false");
        kv("sri_streaming", s.health.sri_streaming ? "true" : "false");
        kv("tool_synced", s.health.tool_synced ? "true" : "false");
        kv("active_controller", s.active_controller);
        arr.status.push_back(st);
        diag_pub.publish(arr);
      });

  ros::AsyncSpinner spinner(3);  // subs + services + action (decision 13)
  spinner.start();

  if (!runtime->start()) {
    ROS_ERROR("soft_robot_manager: runtime failed to start");
    return 1;
  }
  cal_server->start();

  // Preload both controllers so the READY precondition can latch. Retry
  // in the background until the controller_manager is up.
  std::thread preload([&] {
    ros::Rate r(1.0);
    while (ros::ok()) {
      if (loadController(cfg.compliance_controller) &&
          loadController(cfg.correction_controller)) {
        runtime->setControllersLoaded(true);
        ROS_INFO("soft_robot_manager: controllers loaded");
        return;
      }
      r.sleep();
    }
  });

  ROS_INFO("soft_robot_manager: up (payload file: %s)", payload_file.c_str());
  ros::waitForShutdown();
  runtime->stop();
  preload.join();
  return 0;
}
