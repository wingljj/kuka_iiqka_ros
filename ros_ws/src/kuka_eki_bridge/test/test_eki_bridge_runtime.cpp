#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>

#include "kuka_eki_bridge/eki_bridge_runtime.h"
#include "kuka_eki_bridge/eki_mock_server.h"

using kuka_eki::CommandOutcome;
using kuka_eki::EkiAction;
using kuka_eki::EkiBridgeConfig;
using kuka_eki::EkiBridgeRuntime;
using kuka_eki::EkiMockConfig;
using kuka_eki::EkiMockServer;
using kuka_eki::ExecuteResult;
using kuka_eki::Frame6;

namespace {

bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

EkiBridgeConfig config(std::uint16_t port) {
  EkiBridgeConfig c;
  c.kuka_ip = "127.0.0.1";
  c.eki_port = port;
  c.connect_timeout_ms = 200;
  c.receive_timeout_ms = 20;
  c.reconnect_backoff_s = 0.05;
  c.auto_reconnect = true;
  c.session.response_timeout_s = 0.5;
  c.session.state_timeout_s = 0.2;
  return c;
}

}  // namespace

TEST(EkiRuntime, ExecuteStartRsiAccepted) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const ExecuteResult r = rt.execute(EkiAction::START_RSI);
  EXPECT_EQ(r.outcome, CommandOutcome::ACCEPTED);
  EXPECT_TRUE(r.state.rsi_active);
  EXPECT_TRUE(mock.rsiActive());
  rt.stop();
}

TEST(EkiRuntime, RejectedPropagatesKrcErrorCode) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.injectFault();
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const ExecuteResult r = rt.execute(EkiAction::START_RSI);
  EXPECT_EQ(r.outcome, CommandOutcome::REJECTED);
  EXPECT_EQ(r.error_code, kuka_eki::kErrFaulted);
  rt.stop();
}

TEST(EkiRuntime, ResetFaultThenStartSucceeds) {
  // Plan 2 follow-up 5 (KRC-side share): fault -> reset -> start workflow.
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.injectFault();
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  EXPECT_EQ(rt.execute(EkiAction::RESET_FAULT).outcome,
            CommandOutcome::ACCEPTED);
  EXPECT_EQ(rt.execute(EkiAction::START_RSI).outcome,
            CommandOutcome::ACCEPTED);
  rt.stop();
}

TEST(EkiRuntime, TimeoutIsBoundedAndTerminal) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setRespondToNext(false);  // swallow the command
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const auto t0 = std::chrono::steady_clock::now();
  const ExecuteResult r = rt.execute(EkiAction::QUERY_STATE);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
  EXPECT_EQ(r.outcome, CommandOutcome::TIMEOUT);
  EXPECT_LT(ms, 2000);  // response_timeout 0.5 s + margin, well bounded
  rt.stop();
}

TEST(EkiRuntime, GetToolDeliversToolFrame) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  mock.setTool(10.5, 0.0, 235.0, 0.0, 90.0, 0.0);
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  const ExecuteResult r = rt.execute(EkiAction::GET_TOOL);
  ASSERT_EQ(r.outcome, CommandOutcome::ACCEPTED);
  EXPECT_NEAR(r.state.tool.x, 10.5, 1e-6);
  EXPECT_NEAR(r.state.tool.z, 235.0, 1e-6);
  EXPECT_NEAR(r.state.tool.b, 90.0, 1e-6);
  rt.stop();
}

TEST(EkiRuntime, NotConnectedRejectsImmediately) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  const std::uint16_t dead_port = mock.port();
  mock.stop();  // nobody listens anymore
  EkiBridgeRuntime rt{config(dead_port)};
  ASSERT_TRUE(rt.start());
  const ExecuteResult r = rt.execute(EkiAction::QUERY_STATE);
  EXPECT_EQ(r.outcome, CommandOutcome::REJECTED);
  EXPECT_EQ(r.error_code, kuka_eki::kErrNotConnected);
  rt.stop();
}

TEST(EkiRuntime, ReconnectsAfterDropAndServesCommands) {
  EkiMockServer mock{EkiMockConfig{}};
  ASSERT_TRUE(mock.start());
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 2000));
  ASSERT_EQ(rt.execute(EkiAction::QUERY_STATE).outcome,
            CommandOutcome::ACCEPTED);
  mock.dropClient();
  ASSERT_TRUE(waitFor([&] { return rt.status().reconnects >= 1; }, 2000));
  EXPECT_EQ(rt.execute(EkiAction::QUERY_STATE).outcome,
            CommandOutcome::ACCEPTED);
  rt.stop();
}

TEST(EkiRuntime, HeartbeatFreshnessAndBoundedStop) {
  EkiMockConfig mc;
  mc.heartbeat_period_s = 0.02;
  EkiMockServer mock{mc};
  ASSERT_TRUE(mock.start());
  EkiBridgeRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(
      waitFor([&] { return rt.status().session.state_fresh; }, 2000));
  const auto t0 = std::chrono::steady_clock::now();
  rt.stop();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
  EXPECT_LT(ms, 1000);
}

TEST(EkiRuntime, ConnectNowTimeoutIsSingleShot) {
  // Plan 4 follow-up 5 (N2) ruling: a failed manual connect must not arm
  // an endless background retry loop when auto_reconnect is off.
  std::uint16_t port = 0;
  {
    EkiMockServer probe{EkiMockConfig{}};
    ASSERT_TRUE(probe.start());
    port = probe.port();
    probe.stop();
  }
  EkiBridgeConfig c = config(port);
  c.auto_reconnect = false;
  c.connect_timeout_ms = 100;
  EkiBridgeRuntime rt{c};
  ASSERT_TRUE(rt.start());

  EXPECT_FALSE(rt.connectNow(300));  // nobody listening: bounded failure
  // Drain the one attempt that may still be in flight at withdrawal time.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EkiMockConfig mock_cfg;
  mock_cfg.listen_port = port;
  EkiMockServer mock{mock_cfg};
  ASSERT_TRUE(mock.start());
  // Withdrawn request + auto_reconnect=false: must NOT connect by itself.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_FALSE(rt.status().connected);

  EXPECT_TRUE(rt.connectNow(2000));  // an explicit new request connects
  rt.stop();
  mock.stop();
}
