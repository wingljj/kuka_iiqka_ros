#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "kuka_eki_bridge/eki_frame.h"
#include "kuka_eki_bridge/eki_mock_server.h"
#include "kuka_eki_bridge/eki_stream_splitter.h"
#include "kuka_eki_bridge/tcp_client_transport.h"

using kuka_eki::EkiAction;
using kuka_eki::EkiCommand;
using kuka_eki::EkiMockConfig;
using kuka_eki::EkiMockServer;
using kuka_eki::EkiStateFrame;
using kuka_eki::EkiStreamSplitter;
using kuka_eki::TcpClientTransport;

namespace {

bool sendCommand(TcpClientTransport& t, std::uint32_t seq, EkiAction action,
                 int value = 0) {
  EkiCommand c;
  c.seq = seq;
  c.action = action;
  c.value = value;
  char buf[1024];
  const std::size_t n = kuka_eki::serializeCommand(c, buf, sizeof(buf));
  return n > 0 && t.send(buf, n, 200);
}

// Bounded wait for the next parsed state frame. Uses the real splitter +
// parser, so this loop is also an end-to-end schema check.
bool nextState(TcpClientTransport& t, EkiStreamSplitter& splitter,
               EkiStateFrame& out, int deadline_ms = 1000) {
  bool got = false;
  char buf[2048];
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(deadline_ms);
  while (!got && std::chrono::steady_clock::now() < deadline) {
    const int n = t.receive(buf, sizeof(buf), 50);
    if (n < 0) return false;
    if (n == 0) continue;
    splitter.feed(buf, static_cast<std::size_t>(n),
                  [&](const char* d, std::size_t len) {
                    if (!got) got = kuka_eki::parseState(d, len, out);
                  });
  }
  return got;
}

}  // namespace

TEST(EkiMock, StartRsiWorkflowAcksAndSetsActive) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 1, EkiAction::START_RSI));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_EQ(s.ack_seq, 1u);
  EXPECT_TRUE(s.ack_ok);
  EXPECT_TRUE(s.rsi_active);
  EXPECT_TRUE(mock.rsiActive());
}

TEST(EkiMock, StartRsiRefusedWhenNotReady) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setReady(false);
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 2, EkiAction::START_RSI));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_FALSE(s.ack_ok);
  EXPECT_EQ(s.ack_code, kuka_eki::kErrNotReady);
  EXPECT_FALSE(mock.rsiActive());
}

TEST(EkiMock, FaultThenResetWorkflow) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.injectFault();
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(sendCommand(t, 3, EkiAction::START_RSI));
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_FALSE(s.ack_ok);
  EXPECT_EQ(s.ack_code, kuka_eki::kErrFaulted);
  ASSERT_TRUE(sendCommand(t, 4, EkiAction::RESET_FAULT));
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_FALSE(s.fault);
  ASSERT_TRUE(sendCommand(t, 5, EkiAction::START_RSI));
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_TRUE(s.rsi_active);
}

TEST(EkiMock, ToolQueryReturnsConfiguredTool) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setTool(10.5, 0.0, 235.0, 0.0, 90.0, 0.0);
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 6, EkiAction::GET_TOOL));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_NEAR(s.tool.x, 10.5, 1e-6);
  EXPECT_NEAR(s.tool.z, 235.0, 1e-6);
  EXPECT_NEAR(s.tool.b, 90.0, 1e-6);
}

TEST(EkiMock, SetModeUpdatesHeartbeatState) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 7, EkiAction::SET_MODE, 2));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_TRUE(s.ack_ok);
  EXPECT_EQ(s.mode, 2);
  EXPECT_EQ(mock.mode(), 2);
  mock.pushHeartbeat();
  ASSERT_TRUE(nextState(t, sp, s));
  EXPECT_EQ(s.ack_seq, 0u);  // heartbeat marker
  EXPECT_EQ(s.mode, 2);
}

TEST(EkiMock, SwallowedCommandProducesNoAck) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setRespondToNext(false);
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  ASSERT_TRUE(sendCommand(t, 8, EkiAction::QUERY_STATE));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  EXPECT_FALSE(nextState(t, sp, s, 200));  // no answer within the window
  EXPECT_EQ(mock.commandsReceived(), 1u);
}

TEST(EkiMock, MalformedXmlIsSurvivable) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.sendMalformed();  // "<RobotState><Broken></RobotState>" variant
  ASSERT_TRUE(sendCommand(t, 9, EkiAction::QUERY_STATE));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  ASSERT_TRUE(nextState(t, sp, s));  // parser skipped the bad doc
  EXPECT_EQ(s.ack_seq, 9u);
}

TEST(EkiMock, HeartbeatModeStreamsPeriodically) {
  EkiMockConfig cfg;
  cfg.heartbeat_period_s = 0.02;
  EkiMockServer mock{cfg};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  EkiStreamSplitter sp;
  EkiStateFrame s;
  int got = 0;
  for (int i = 0; i < 3; ++i)
    if (nextState(t, sp, s, 500)) ++got;
  EXPECT_EQ(got, 3);
}
