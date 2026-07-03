#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_frame.h"
#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

using kuka_rsi::MockConfig;
using kuka_rsi::RobFrame;
using kuka_rsi::RsiMockCore;
using kuka_rsi::RsiMockServer;
using kuka_rsi::SenFrame;
using kuka_rsi::UdpTransport;

namespace {
std::string senReply(std::uint64_t ipoc, double x = 0.0, double a = 0.0) {
  SenFrame f;
  f.x = x;
  f.a = a;
  f.ipoc = ipoc;
  f.watchdog = 7;
  char buf[1024];
  const std::size_t n = kuka_rsi::serializeSenFrame(f, buf, sizeof(buf));
  return std::string(buf, n);
}
}  // namespace

TEST(MockCore, StateFrameIsParsableAndIpocAdvances) {
  MockConfig cfg;
  cfg.x0 = 100.0;
  cfg.ipoc_start = 1000;
  cfg.ipoc_step = 4;
  RsiMockCore core(cfg);

  char buf[1024];
  ASSERT_GT(core.buildStateFrame(buf, sizeof(buf)), 0u);
  RobFrame f;
  ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, std::strlen(buf), f));
  EXPECT_DOUBLE_EQ(f.x, 100.0);
  EXPECT_DOUBLE_EQ(f.z, 800.0);
  EXPECT_EQ(f.ipoc, 1000u);

  ASSERT_GT(core.buildStateFrame(buf, sizeof(buf)), 0u);
  ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, std::strlen(buf), f));
  EXPECT_EQ(f.ipoc, 1004u);
}

TEST(MockCore, CorrectEchoIntegratesCorrection) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  core.buildStateFrame(buf, sizeof(buf));  // sends ipoc_start = 1000
  const std::string reply = senReply(1000, 0.5, 0.01);
  ASSERT_TRUE(core.applyReply(reply.data(), reply.size()));
  EXPECT_DOUBLE_EQ(core.x(), 0.5);
  EXPECT_DOUBLE_EQ(core.a(), 0.01);
  EXPECT_EQ(core.stats().ipoc_echo_errors, 0u);
  EXPECT_EQ(core.stats().last_watchdog, 7u);
}

TEST(MockCore, WrongEchoCountsErrorAndDoesNotIntegrate) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  core.buildStateFrame(buf, sizeof(buf));  // ipoc 1000
  const std::string reply = senReply(999, 5.0);
  EXPECT_FALSE(core.applyReply(reply.data(), reply.size()));
  EXPECT_DOUBLE_EQ(core.x(), 0.0);
  EXPECT_EQ(core.stats().ipoc_echo_errors, 1u);
}

TEST(MockCore, MalformedReplyCountsParseError) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  core.buildStateFrame(buf, sizeof(buf));
  EXPECT_FALSE(core.applyReply("garbage", 7));
  EXPECT_EQ(core.stats().parse_errors, 1u);
}

TEST(MockCore, CorrectionsAccumulateAcrossCycles) {
  RsiMockCore core(MockConfig{});
  char buf[1024];
  for (int i = 0; i < 3; ++i) {
    core.buildStateFrame(buf, sizeof(buf));
    RobFrame f;
    ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, std::strlen(buf), f));
    const std::string reply = senReply(f.ipoc, 0.1);
    ASSERT_TRUE(core.applyReply(reply.data(), reply.size()));
  }
  EXPECT_NEAR(core.x(), 0.3, 1e-12);
  EXPECT_EQ(core.stats().frames_sent, 3u);
  EXPECT_EQ(core.stats().replies_received, 3u);
}

TEST(MockServer, DrivesCycleAgainstManualPeer) {
  UdpTransport pc;  // hand-rolled PC side
  ASSERT_TRUE(pc.bind("127.0.0.1", 0));

  MockConfig cfg;
  cfg.x0 = 50.0;
  RsiMockServer server(cfg, "127.0.0.1", pc.boundPort(), 100);
  ASSERT_TRUE(server.start());

  // Answer 5 cycles with a +0.2 mm X correction each.
  char buf[1024];
  for (int i = 0; i < 5; ++i) {
    const int n = pc.receive(buf, sizeof(buf), 500);
    ASSERT_GT(n, 0);
    RobFrame f;
    ASSERT_TRUE(kuka_rsi::parseRobFrame(buf, n, f));
    const std::string reply = senReply(f.ipoc, 0.2);
    ASSERT_TRUE(pc.sendToLastSender(reply.data(), reply.size()));
  }
  server.stop();

  const kuka_rsi::MockStats s = server.statsSnapshot();
  EXPECT_GE(s.replies_received, 5u);
  EXPECT_EQ(s.ipoc_echo_errors, 0u);
}

TEST(MockServer, CountsReplyTimeouts) {
  RsiMockServer server(MockConfig{}, "127.0.0.1", 1 /* nobody listens */, 10);
  ASSERT_TRUE(server.start());
  // Bounded wait: 3 timeouts at 10 ms each should arrive well within 2 s.
  for (int i = 0; i < 200 && server.statsSnapshot().reply_timeouts < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  server.stop();
  EXPECT_GE(server.statsSnapshot().reply_timeouts, 3u);
  EXPECT_EQ(server.statsSnapshot().replies_received, 0u);
}
