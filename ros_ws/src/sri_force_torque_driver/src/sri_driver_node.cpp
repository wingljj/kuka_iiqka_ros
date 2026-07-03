// Thin ROS shell around SriDriverRuntime (spec 5.6). All protocol/session
// logic lives in the offline-tested library; this file only loads
// parameters and forwards data. Topics/services use the absolute names
// fixed by the spec (/sri_ft/...).
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <soft_robot_msgs/SetFilter.h>
#include <soft_robot_msgs/SriStatus.h>
#include <std_srvs/Trigger.h>

#include <string>

#include "sri_force_torque_driver/sri_driver_runtime.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sri_ft_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sri::SriDriverConfig cfg;
  int port = 4008;
  int zero_timeout_ms = 3000;
  std::string frame_id;
  pnh.param<std::string>("sensor_ip", cfg.sensor_ip, "192.168.1.1");
  pnh.param("sensor_port", port, 4008);
  cfg.sensor_port = static_cast<std::uint16_t>(port);
  pnh.param("connect_timeout_ms", cfg.connect_timeout_ms, 1000);
  pnh.param("receive_timeout_ms", cfg.receive_timeout_ms, 50);
  pnh.param("reconnect_backoff_s", cfg.reconnect_backoff_s, 0.5);
  pnh.param("sample_timeout_s", cfg.session.sample_timeout_s, 0.1);
  pnh.param("nominal_rate_hz", cfg.session.nominal_rate_hz, 250.0);
  pnh.param("zero_sample_count", cfg.session.zero_sample_count, 100);
  pnh.param("filter_cutoff_hz", cfg.session.filter_cutoff_hz, 0.0);
  pnh.param("bias_limit_n", cfg.session.bias_limit_n, 120.0);
  pnh.param("zero_timeout_ms", zero_timeout_ms, 3000);
  pnh.param<std::string>("frame_id", frame_id, "sri_ft_link");

  ros::Publisher wrench_pub =
      nh.advertise<geometry_msgs::WrenchStamped>("/sri_ft/wrench_raw", 10);
  ros::Publisher status_pub =
      nh.advertise<soft_robot_msgs::SriStatus>("/sri_ft/status", 10);

  sri::SriDriverRuntime runtime(cfg);
  runtime.setSampleCallback([&](const sri::SriWrenchSample& s) {
    geometry_msgs::WrenchStamped msg;
    // Reception instant: this callback runs synchronously in the rx
    // thread right after the socket read (Plan 3 follow-up 3). Never a
    // zero stamp.
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = s.w.fx;
    msg.wrench.force.y = s.w.fy;
    msg.wrench.force.z = s.w.fz;
    msg.wrench.torque.x = s.w.tx;
    msg.wrench.torque.y = s.w.ty;
    msg.wrench.torque.z = s.w.tz;
    wrench_pub.publish(msg);
  });

  ros::ServiceServer zero_srv = nh.advertiseService<std_srvs::Trigger::Request,
                                                    std_srvs::Trigger::Response>(
      "/sri_ft/zero",
      [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        res.success = runtime.requestZero(zero_timeout_ms);
        res.message = res.success
                          ? "tare captured"
                          : "tare failed: no stream or bias above limit";
        return true;
      });
  ros::ServiceServer filter_srv =
      nh.advertiseService<soft_robot_msgs::SetFilter::Request,
                          soft_robot_msgs::SetFilter::Response>(
          "/sri_ft/set_filter",
          [&](soft_robot_msgs::SetFilter::Request& req,
              soft_robot_msgs::SetFilter::Response& res) {
            res.success = runtime.setFilterCutoff(req.cutoff_hz);
            res.message =
                res.success ? "filter updated" : "cutoff_hz must be >= 0";
            return true;
          });

  ros::Timer status_timer =
      nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) {
        const sri::SriDriverStatus st = runtime.status();
        soft_robot_msgs::SriStatus msg;
        msg.header.stamp = ros::Time::now();
        msg.connected = st.connected;
        msg.streaming = st.session.streaming;
        msg.reconnects = st.reconnects;
        msg.samples = st.session.samples;
        msg.bad_frames = st.session.bad_frames;
        msg.package_gaps = st.session.package_gaps;
        msg.zero_rejects = st.session.zero_rejects;
        msg.last_sample_age = st.session.last_sample_age_s;
        msg.zero_active = st.session.zero_active;
        msg.filter_cutoff_hz = st.session.filter_cutoff_hz;
        status_pub.publish(msg);
      });

  if (!runtime.start()) {
    ROS_ERROR("sri_ft_driver: runtime failed to start");
    return 1;
  }
  ROS_INFO("sri_ft_driver: connecting to %s:%u", cfg.sensor_ip.c_str(),
           cfg.sensor_port);
  ros::spin();
  runtime.stop();
  return 0;
}
