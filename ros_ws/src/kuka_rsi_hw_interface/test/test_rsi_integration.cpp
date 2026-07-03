#include <gtest/gtest.h>

#include "kuka_rsi_hw_interface/kuka_rsi_robot_hw.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"

using kuka_rsi::HwConfig;
using kuka_rsi::KukaRsiRobotHW;
using kuka_rsi::MockConfig;
using kuka_rsi::MockStats;
using kuka_rsi::RsiMockServer;

namespace {

HwConfig hwConfig() {
  HwConfig cfg;
  cfg.listen_ip = "127.0.0.1";
  cfg.listen_port = 0;
  cfg.read_timeout_ms = 100;  // generous: mock thread scheduling jitter
  cfg.max_consecutive_timeouts = 5;
  return cfg;
}

// One PC-side control cycle: read state, apply a command, write reply.
void spinCycle(KukaRsiRobotHW& hw, double cmd_x, double cmd_a = 0.0) {
  hw.read(ros::Time(), ros::Duration(0.004));
  auto h = hw.get<kuka_rsi::CartesianCorrectionCommandInterface>()
               ->getHandle("kuka_tcp");
  h.setCommand(cmd_x, 0, 0, cmd_a, 0, 0);
  hw.write(ros::Time(), ros::Duration(0.004));
}

}  // namespace

TEST(RsiIntegration, ClosedLoopEchoAndIntegration) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));

  MockConfig mock_cfg;
  mock_cfg.x0 = 500.0;
  RsiMockServer mock(mock_cfg, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  // 50 cycles of +0.2 mm X corrections.
  for (int i = 0; i < 50; ++i) spinCycle(hw, 0.2);
  mock.stop();

  const MockStats s = mock.statsSnapshot();
  EXPECT_EQ(s.ipoc_echo_errors, 0u);   // every reply echoed the right IPOC
  EXPECT_EQ(s.parse_errors, 0u);       // every reply was well-formed
  EXPECT_GE(s.replies_received, 50u);
  EXPECT_TRUE(hw.connected());
  EXPECT_FALSE(hw.faulted());
  EXPECT_EQ(hw.sessionStats().ipoc_jumps, 0u);

  // The mock integrated our corrections: pose moved by ~50 * 0.2 mm.
  // (>= 45 tolerates a few cycles lost to thread startup/shutdown.)
  EXPECT_GE(mock.poseX(), 500.0 + 45 * 0.2);
}

TEST(RsiIntegration, StateInterfaceTracksInjectedPose) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  RsiMockServer mock(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  spinCycle(hw, 0.0);  // connect
  mock.setPose(111.0, 222.0, 333.0, 10.0, 20.0, 30.0);
  // A few cycles so a post-injection frame is definitely consumed.
  for (int i = 0; i < 5; ++i) spinCycle(hw, 0.0);
  mock.stop();

  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_DOUBLE_EQ(h.getX(), 111.0);
  EXPECT_DOUBLE_EQ(h.getY(), 222.0);
  EXPECT_DOUBLE_EQ(h.getA(), 10.0);
  EXPECT_DOUBLE_EQ(h.getC(), 30.0);
}

TEST(RsiIntegration, FaultZeroOutputObservedByMock) {
  HwConfig cfg = hwConfig();
  cfg.read_timeout_ms = 10;
  cfg.max_consecutive_timeouts = 2;
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(cfg));

  RsiMockServer mock(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());
  spinCycle(hw, 0.2);  // healthy cycle
  mock.stop();         // KRC dies

  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 1
  hw.read(ros::Time(), ros::Duration(0.004));  // timeout 2 -> latched fault
  ASSERT_TRUE(hw.faulted());

  // KRC comes back, but the PC fault is latched: replies must be zero+Stop.
  RsiMockServer mock2(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock2.start());
  for (int i = 0; i < 5; ++i) spinCycle(hw, 0.2);
  mock2.stop();

  const MockStats s = mock2.statsSnapshot();
  EXPECT_EQ(s.last_stop, 1);           // Stop requested
  EXPECT_DOUBLE_EQ(mock2.poseX(), 0.0);  // zero correction: pose unmoved
  EXPECT_TRUE(hw.faulted());           // still latched

  hw.resetFault();
  EXPECT_FALSE(hw.faulted());
}

TEST(RsiIntegration, RotationCorrectionsIntegrateOnMock) {
  KukaRsiRobotHW hw;
  ASSERT_TRUE(hw.configure(hwConfig()));
  RsiMockServer mock(MockConfig{}, "127.0.0.1", hw.listenPort(), 200);
  ASSERT_TRUE(mock.start());

  for (int i = 0; i < 20; ++i) spinCycle(hw, 0.0, 0.05);  // A axis, at limit
  mock.stop();

  EXPECT_EQ(mock.statsSnapshot().ipoc_echo_errors, 0u);
  // 20 cycles * 0.05 deg = 1.0 deg total on A (tolerate a few lost cycles).
  auto h = hw.get<kuka_rsi::CartesianStateInterface>()->getHandle("kuka_tcp");
  EXPECT_GE(h.getA(), 0.7);
  EXPECT_LE(h.getA(), 1.0 + 1e-9);
}
