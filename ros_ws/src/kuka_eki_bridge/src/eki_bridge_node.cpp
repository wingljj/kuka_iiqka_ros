// Thin ROS shell around EkiBridgeRuntime (spec 5.2, 6.2, 6.4). All
// protocol/session logic lives in the offline-tested library; this file
// only loads parameters and forwards service calls. Service/topic names
// are the absolute ones fixed by the spec (/kuka/eki/...).
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <soft_robot_msgs/EkiState.h>
#include <soft_robot_msgs/GetTool.h>
#include <soft_robot_msgs/SetEkiMode.h>
#include <soft_robot_msgs/SetToolBase.h>
#include <std_srvs/Trigger.h>

#include <string>

#include "kuka_eki_bridge/eki_bridge_runtime.h"

namespace {

std::string outcomeMessage(const kuka_eki::ExecuteResult& r) {
  switch (r.outcome) {
    case kuka_eki::CommandOutcome::ACCEPTED:
      return "ok";
    case kuka_eki::CommandOutcome::REJECTED:
      return r.error_code == kuka_eki::kErrNotConnected
                 ? "not connected to the KRC"
                 : "rejected by KRC (code " + std::to_string(r.error_code) +
                       ")";
    case kuka_eki::CommandOutcome::TIMEOUT:
      return "KRC response timeout";
    default:
      return "no result";
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "kuka_eki_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  kuka_eki::EkiBridgeConfig cfg;
  int port = 54600;
  int connect_wait_ms = 3000;
  pnh.param<std::string>("kuka_ip", cfg.kuka_ip, "192.168.1.10");
  pnh.param("eki_port", port, 54600);
  cfg.eki_port = static_cast<std::uint16_t>(port);
  pnh.param("connect_timeout_ms", cfg.connect_timeout_ms, 1000);
  pnh.param("receive_timeout_ms", cfg.receive_timeout_ms, 50);
  pnh.param("reconnect_backoff_s", cfg.reconnect_backoff_s, 1.0);
  pnh.param("auto_reconnect", cfg.auto_reconnect, true);
  pnh.param("response_timeout_s", cfg.session.response_timeout_s, 2.0);
  pnh.param("state_timeout_s", cfg.session.state_timeout_s, 1.0);
  pnh.param("connect_wait_ms", connect_wait_ms, 3000);

  kuka_eki::EkiBridgeRuntime runtime(cfg);

  auto trigger = [&runtime](kuka_eki::EkiAction action) {
    return [&runtime, action](std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
      const kuka_eki::ExecuteResult r = runtime.execute(action);
      res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
      res.message = outcomeMessage(r);
      return true;
    };
  };

  ros::ServiceServer connect_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/connect",
          [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
            res.success = runtime.connectNow(connect_wait_ms);
            res.message = res.success ? "connected" : "connect timeout";
            return true;
          });
  ros::ServiceServer start_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/start_rsi_program",
          trigger(kuka_eki::EkiAction::START_RSI));
  ros::ServiceServer stop_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/stop_rsi_program",
          trigger(kuka_eki::EkiAction::STOP_RSI));
  ros::ServiceServer reset_srv =
      nh.advertiseService<std_srvs::Trigger::Request,
                          std_srvs::Trigger::Response>(
          "/kuka/eki/reset_fault",
          trigger(kuka_eki::EkiAction::RESET_FAULT));
  ros::ServiceServer mode_srv =
      nh.advertiseService<soft_robot_msgs::SetEkiMode::Request,
                          soft_robot_msgs::SetEkiMode::Response>(
          "/kuka/eki/set_mode",
          [&](soft_robot_msgs::SetEkiMode::Request& req,
              soft_robot_msgs::SetEkiMode::Response& res) {
            const kuka_eki::ExecuteResult r = runtime.execute(
                kuka_eki::EkiAction::SET_MODE, static_cast<int>(req.mode));
            res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
            res.message = outcomeMessage(r);
            return true;
          });
  ros::ServiceServer tool_base_srv =
      nh.advertiseService<soft_robot_msgs::SetToolBase::Request,
                          soft_robot_msgs::SetToolBase::Response>(
          "/kuka/eki/set_tool_base",
          [&](soft_robot_msgs::SetToolBase::Request& req,
              soft_robot_msgs::SetToolBase::Response& res) {
            kuka_eki::Frame6 tool{req.tool_x, req.tool_y, req.tool_z,
                                  req.tool_a, req.tool_b, req.tool_c};
            kuka_eki::Frame6 base{req.base_x, req.base_y, req.base_z,
                                  req.base_a, req.base_b, req.base_c};
            const kuka_eki::ExecuteResult r = runtime.execute(
                kuka_eki::EkiAction::SET_TOOL_BASE, 0, tool, base);
            res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
            res.message = outcomeMessage(r);
            return true;
          });
  ros::ServiceServer get_tool_srv =
      nh.advertiseService<soft_robot_msgs::GetTool::Request,
                          soft_robot_msgs::GetTool::Response>(
          "/kuka/eki/get_tool",
          [&](soft_robot_msgs::GetTool::Request&,
              soft_robot_msgs::GetTool::Response& res) {
            const kuka_eki::ExecuteResult r =
                runtime.execute(kuka_eki::EkiAction::GET_TOOL);
            res.success = r.outcome == kuka_eki::CommandOutcome::ACCEPTED;
            res.message = outcomeMessage(r);
            res.x = r.state.tool.x;
            res.y = r.state.tool.y;
            res.z = r.state.tool.z;
            res.a = r.state.tool.a;
            res.b = r.state.tool.b;
            res.c = r.state.tool.c;
            return true;
          });

  ros::Publisher state_pub =
      nh.advertise<soft_robot_msgs::EkiState>("/kuka/eki/state", 10);
  ros::Publisher diag_pub =
      nh.advertise<diagnostic_msgs::DiagnosticArray>("/kuka/diagnostics", 10);

  ros::Timer state_timer =
      nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) {
        const kuka_eki::EkiBridgeStatus st = runtime.status();
        soft_robot_msgs::EkiState msg;
        msg.header.stamp = ros::Time::now();
        msg.connected = st.connected;
        msg.state_fresh = st.session.state_fresh;
        msg.program_ready = st.session.last_state.ready;
        msg.rsi_active = st.session.last_state.rsi_active;
        msg.fault = st.session.last_state.fault;
        msg.mode = static_cast<std::uint8_t>(st.session.last_state.mode);
        msg.reconnects = st.reconnects;
        msg.state_age = st.session.state_age_s;
        msg.tool_x = st.session.last_state.tool.x;
        msg.tool_y = st.session.last_state.tool.y;
        msg.tool_z = st.session.last_state.tool.z;
        msg.tool_a = st.session.last_state.tool.a;
        msg.tool_b = st.session.last_state.tool.b;
        msg.tool_c = st.session.last_state.tool.c;
        state_pub.publish(msg);
      });
  ros::Timer diag_timer =
      nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        const kuka_eki::EkiBridgeStatus st = runtime.status();
        diagnostic_msgs::DiagnosticArray arr;
        arr.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus s;
        s.name = "kuka_eki_bridge: management link";
        s.hardware_id = cfg.kuka_ip;
        if (!st.connected) {
          s.level = diagnostic_msgs::DiagnosticStatus::ERROR;
          s.message = "EKI disconnected";
        } else if (st.session.last_state.fault) {
          s.level = diagnostic_msgs::DiagnosticStatus::ERROR;
          s.message = "KRC fault latched";
        } else if (!st.session.state_fresh) {
          s.level = diagnostic_msgs::DiagnosticStatus::WARN;
          s.message = "state heartbeat stale";
        } else {
          s.level = diagnostic_msgs::DiagnosticStatus::OK;
          s.message = "ok";
        }
        auto kv = [&s](const std::string& k, const std::string& v) {
          diagnostic_msgs::KeyValue e;
          e.key = k;
          e.value = v;
          s.values.push_back(e);
        };
        kv("connected", st.connected ? "true" : "false");
        kv("state_age_s", std::to_string(st.session.state_age_s));
        kv("timeouts", std::to_string(st.session.timeouts));
        kv("bad_frames", std::to_string(st.session.bad_frames));
        kv("reconnects", std::to_string(st.reconnects));
        arr.status.push_back(s);
        diag_pub.publish(arr);
      });

  if (!runtime.start()) {
    ROS_ERROR("kuka_eki_bridge: runtime failed to start");
    return 1;
  }
  ROS_INFO("kuka_eki_bridge: KRC EKI server %s:%u (external system = client)",
           cfg.kuka_ip.c_str(), cfg.eki_port);
  ros::spin();
  runtime.stop();
  return 0;
}
