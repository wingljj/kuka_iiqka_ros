// ROS node shell: runs KukaRsiRobotHW under controller_manager at the RSI
// cycle and publishes link diagnostics. All protocol behavior is covered by
// the offline gtests; this file only wires the pieces to ROS.
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <soft_robot_msgs/RsiState.h>
#include <std_srvs/Trigger.h>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "kuka_rsi_hw_interface");
  ros::NodeHandle root_nh;
  ros::NodeHandle robot_hw_nh("~");

  kuka_rsi::KukaRsiRobotHW hw;
  if (!hw.init(root_nh, robot_hw_nh)) {
    ROS_FATAL("kuka_rsi_hw_interface: init failed (UDP bind?)");
    return 1;
  }
  ROS_INFO("kuka_rsi_hw_interface: listening on UDP port %u", hw.listenPort());

  controller_manager::ControllerManager cm(&hw, root_nh);
  ros::Publisher state_pub =
      root_nh.advertise<soft_robot_msgs::RsiState>("kuka/rsi/state", 10);

  // Manager-facing recovery service (Plan 4 follow-up 10). The clear is
  // deferred to the next read() cycle; cumulative RsiState counters are
  // preserved (clearFault, not reset).
  ros::ServiceServer reset_srv =
      root_nh.advertiseService<std_srvs::Trigger::Request,
                               std_srvs::Trigger::Response>(
          "/kuka/rsi/reset_fault",
          [&hw](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
            hw.requestFaultClear();
            res.success = true;
            res.message = "fault clear requested; applied on next RSI read cycle";
            return true;
          });

  ros::AsyncSpinner spinner(1);  // services/topics off the control thread
  spinner.start();

  // The loop is paced by the KRC: read() blocks (bounded) on its frame.
  ros::Time last = ros::Time::now();
  ros::Time last_pub = last;
  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last;
    last = now;

    hw.read(now, period);
    cm.update(now, period);
    hw.write(now, period);

    if ((now - last_pub).toSec() >= 0.02) {  // 50 Hz diagnostics
      last_pub = now;
      const kuka_rsi::SessionStats& s = hw.sessionStats();
      soft_robot_msgs::RsiState msg;
      msg.header.stamp = now;
      msg.connected = s.connected;
      msg.fault = s.fault;
      msg.ipoc = s.last_ipoc;
      msg.total_timeouts = s.total_timeouts;
      msg.consecutive_timeouts = s.consecutive_timeouts;
      msg.bad_frames = s.bad_frames;
      msg.ipoc_jumps = s.ipoc_jumps;
      msg.saturation_count = hw.saturationCount();
      state_pub.publish(msg);
    }
  }
  return 0;
}
