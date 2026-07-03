#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"
#include "kuka_rsi_hw_interface/rsi_frame.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

using kuka_rsi::HwConfig;
using kuka_rsi::KukaRsiRobotHW;
using kuka_rsi::RobFrame;
using kuka_rsi::UdpTransport;

namespace {

// Hand-rolled KRC peer: gives full control over frame content and timing.
class FakeKrc {
 public:
  bool bind() { return udp_.bind("127.0.0.1", 0); }

  bool sendState(std::uint16_t hw_port, double x, double a1_deg,
                 std::uint64_t ipoc) {
    char buf[1024];
    const int n = std::snprintf(
        buf, sizeof(buf),
        "<Rob Type=\"KUKA\">"
        "<RIst X=\"%.4f\" Y=\"0.0\" Z=\"800.0\" A=\"0.0\" B=\"0.0\" "
        "C=\"0.0\"/>"
        "<AIPos A1=\"%.4f\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" "
        "A6=\"0.0\"/>"
        "<IPOC>%llu</IPOC>"
        "</Rob>",
        x, a1_deg, static_cast<unsigned long long>(ipoc));
    return udp_.sendTo("127.0.0.1", hw_port, buf,
                       static_cast<std::size_t>(n));
  }

  bool sendGarbage(std::uint16_t hw_port) {
    return udp_.sendTo("127.0.0.1", hw_port, "not xml", 7);
  }

  // Receives the hw interface's <Sen> reply and parses key fields.
  bool receiveReply(double& rkorr_x, int& stop, std::uint64_t& ipoc,
                    int timeout_ms = 500) {
    char buf[1024];
    const int n = udp_.receive(buf, sizeof(buf), timeout_ms);
    if (n <= 0) return false;
    const std::string s(buf, n);
    double a, b, c, y, z;
    unsigned long long wd, ip;
    if (std::sscanf(s.c_str(),
                    "<Sen Type=\"ROS\">"
                    "<RKorr X=\"%lf\" Y=\"%lf\" Z=\"%lf\" A=\"%lf\" "
                    "B=\"%lf\" C=\"%lf\"/>"
                    "<Stop S=\"%d\"/>"
                    "<Watchdog W=\"%llu\"/>"
                    "<IPOC>%llu</IPOC>",
                    &rkorr_x, &y, &z, &a, &b, &c, &stop, &wd, &ip) != 9) {
      return false;
    }
    ipoc = ip;
    return true;
  }

 private:
  UdpTransport udp_;
};

HwConfig testConfig(unsigned max_timeouts = 3) {
  HwConfig cfg;
  cfg.listen_ip = "127.0.0.1";
  cfg.listen_port = 0;         // kernel-assigned, collision-free
  cfg.read_timeout_ms = 20;    // keep tests fast
  cfg.max_consecutive_timeouts = max_timeouts;
  return cfg;
}

constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

TEST(RobotHW, ConfigureRegistersAllInterfaces) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  auto* js = hw.get<hardware_interface::JointStateInterface>();
  ASSERT_NE(js, nullptr);
  EXPECT_NO_THROW(js->getHandle("joint_a1"));
  EXPECT_NO_THROW(js->getHandle("joint_a6"));
  auto* cs = hw.get<kuka_rsi::CartesianStateInterface>();
  ASSERT_NE(cs, nullptr);
  EXPECT_NO_THROW(cs->getHandle("kuka_tcp"));
  auto* cc = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>();
  ASSERT_NE(cc, nullptr);
  EXPECT_NO_THROW(cc->getHandle("kuka_tcp"));
}

TEST(RobotHW, ReadUpdatesStateFromFrame) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 123.5, 90.0, 1000));

  hw.read(ros::Time(), ros::Duration(0.004));
  EXPECT_TRUE(hw.connected());
  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 123.5);
  EXPECT_DOUBLE_EQ(h.getZ(), 800.0);
  auto j = hw.get<hardware_interface::JointStateInterface>()
               ->getHandle("joint_a1");
  // Deviation from plan: Noetic's JointStateHandle::getPosition() returns
  // double by value (not a pointer), so no dereference here.
  EXPECT_NEAR(j.getPosition(), 90.0 * kDegToRad, 1e-12);
}

TEST(RobotHW, WriteEchoesIpocAndClampsCommand) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 4242));
  hw.read(ros::Time(), ros::Duration(0.004));

  auto cmd = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
                 ->getHandle("kuka_tcp");
  cmd.setCommand(9.0, 0, 0, 0, 0, 0);  // above 0.5 mm default limit
  hw.write(ros::Time(), ros::Duration(0.004));

  double x;
  int stop;
  std::uint64_t ipoc;
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.5);  // secondary clamp engaged
  EXPECT_EQ(stop, 0);
  EXPECT_EQ(ipoc, 4242u);
  EXPECT_EQ(hw.saturationCount(), 1u);
}

TEST(RobotHW, CommandBufferClearedAfterWrite) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 1));
  hw.read(ros::Time(), ros::Duration(0.004));

  auto cmd = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
                 ->getHandle("kuka_tcp");
  cmd.setCommand(0.1, 0, 0, 0, 0, 0);
  hw.write(ros::Time(), ros::Duration(0.004));
  double x;
  int stop;
  std::uint64_t ipoc;
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.1);

  // Second cycle without a fresh setCommand: stale value must not repeat.
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.1, 0.0, 2));
  hw.read(ros::Time(), ros::Duration(0.004));
  hw.write(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.0);
}

TEST(RobotHW, TimeoutsLatchFaultAndForceZeroWithStop) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig(2)));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 10));
  hw.read(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(hw.connected());

  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 1
  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 2 -> fault
  EXPECT_TRUE(hw.faulted());

  auto cmd = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
                 ->getHandle("kuka_tcp");
  cmd.setCommand(0.3, 0, 0, 0, 0, 0);
  hw.write(ros::Time(), ros::Duration(0.004));
  double x;
  int stop;
  std::uint64_t ipoc;
  ASSERT_TRUE(krc.receiveReply(x, stop, ipoc));
  EXPECT_DOUBLE_EQ(x, 0.0);  // faulted: zero correction
  EXPECT_EQ(stop, 1);        // and Stop requested

  hw.resetFault();
  EXPECT_FALSE(hw.faulted());
}

TEST(RobotHW, BadFramesCountTowardFault) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig(2)));
  FakeKrc krc;
  ASSERT_TRUE(krc.bind());
  ASSERT_TRUE(krc.sendState(hw.listenPort(), 0.0, 0.0, 10));
  hw.read(ros::Time(), ros::Duration(0.004));

  ASSERT_TRUE(krc.sendGarbage(hw.listenPort()));
  hw.read(ros::Time(), ros::Duration(0.004));
  ASSERT_TRUE(krc.sendGarbage(hw.listenPort()));
  hw.read(ros::Time(), ros::Duration(0.004));
  EXPECT_TRUE(hw.faulted());
  EXPECT_EQ(hw.sessionStats().bad_frames, 2u);
}

TEST(RobotHW, WriteBeforeAnyFrameIsSilentNoop) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(testConfig()));
  // No frame ever received: nothing to reply to; must not crash.
  hw.write(ros::Time(), ros::Duration(0.004));
  EXPECT_FALSE(hw.connected());
}
