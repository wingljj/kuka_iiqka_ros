#include <gtest/gtest.h>

#include "kuka_eki_bridge/eki_session_core.h"

using kuka_eki::CommandOutcome;
using kuka_eki::EkiSessionConfig;
using kuka_eki::EkiSessionCore;
using kuka_eki::EkiStateFrame;

namespace {

constexpr double kNow = 200.0;  // arbitrary monotonic origin [s]

EkiSessionConfig config() {
  EkiSessionConfig c;
  c.response_timeout_s = 0.2;
  c.state_timeout_s = 0.1;
  return c;
}

EkiStateFrame state(std::uint32_t ack_seq, bool ok = true, int code = 0) {
  EkiStateFrame s;
  s.ack_seq = ack_seq;
  s.ack_ok = ok;
  s.ack_code = code;
  s.ready = true;
  s.mode = 1;
  s.tool.z = 235.0;
  s.valid = true;
  return s;
}

}  // namespace

TEST(EkiSession, HeartbeatRefreshesFreshnessOnly) {
  EkiSessionCore core;
  core.configure(config());
  EXPECT_FALSE(core.snapshot(kNow).state_fresh);
  EXPECT_EQ(core.onState(state(0), kNow), CommandOutcome::NONE);
  EXPECT_TRUE(core.snapshot(kNow + 0.05).state_fresh);
  EXPECT_NEAR(core.snapshot(kNow + 0.05).state_age_s, 0.05, 1e-9);
  EXPECT_FALSE(core.snapshot(kNow + 0.2).state_fresh);  // past 0.1 s
}

TEST(EkiSession, MatchingAckAcceptsCommand) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(5, kNow));
  EXPECT_TRUE(core.commandPending());
  EXPECT_EQ(core.onState(state(5), kNow + 0.01), CommandOutcome::ACCEPTED);
  EXPECT_FALSE(core.commandPending());
  // The ack frame doubles as a state frame: tool data is available.
  EXPECT_NEAR(core.snapshot(kNow + 0.01).last_state.tool.z, 235.0, 1e-9);
}

TEST(EkiSession, NackRejectsCommand) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(6, kNow));
  EXPECT_EQ(core.onState(state(6, false, kuka_eki::kErrNotReady), kNow),
            CommandOutcome::REJECTED);
  EXPECT_EQ(core.snapshot(kNow).last_state.ack_code, kuka_eki::kErrNotReady);
}

TEST(EkiSession, MismatchedAckIsIgnoredByPendingCommand) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(7, kNow));
  EXPECT_EQ(core.onState(state(3), kNow), CommandOutcome::NONE);  // stale ack
  EXPECT_TRUE(core.commandPending());
}

TEST(EkiSession, SecondBeginWhilePendingRefused) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(1, kNow));
  EXPECT_FALSE(core.beginCommand(2, kNow));
}

TEST(EkiSession, PendingCommandTimesOutViaTick) {
  EkiSessionCore core;
  core.configure(config());  // response_timeout_s = 0.2
  ASSERT_TRUE(core.beginCommand(8, kNow));
  EXPECT_EQ(core.tick(kNow + 0.1), CommandOutcome::NONE);
  EXPECT_EQ(core.tick(kNow + 0.25), CommandOutcome::TIMEOUT);
  EXPECT_FALSE(core.commandPending());
  EXPECT_EQ(core.snapshot(kNow + 0.25).timeouts, 1u);
}

TEST(EkiSession, ResetFailsPendingCommandAsTimeout) {
  EkiSessionCore core;
  core.configure(config());
  ASSERT_TRUE(core.beginCommand(9, kNow));
  EXPECT_EQ(core.reset(), CommandOutcome::TIMEOUT);  // disconnect semantics
  EXPECT_FALSE(core.commandPending());
  EXPECT_FALSE(core.snapshot(kNow).state_fresh);
}

TEST(EkiSession, CountersAccumulate) {
  EkiSessionCore core;
  core.configure(config());
  core.onState(state(0), kNow);
  core.onState(state(0), kNow + 0.01);
  core.onBadFrame();
  const auto snap = core.snapshot(kNow + 0.01);
  EXPECT_EQ(snap.states, 2u);
  EXPECT_EQ(snap.bad_frames, 1u);
}
