#include <gtest/gtest.h>

#include "kuka_rsi_hw_interface/rsi_session_monitor.h"

using kuka_rsi::RobFrame;
using kuka_rsi::RsiSessionMonitor;
using kuka_rsi::SessionConfig;

namespace {
RobFrame frameWithIpoc(std::uint64_t ipoc) {
  RobFrame f;
  f.ipoc = ipoc;
  f.valid = true;
  return f;
}
}  // namespace

TEST(SessionMonitor, StartsDisconnectedNoFault) {
  RsiSessionMonitor m{SessionConfig{}};
  EXPECT_FALSE(m.connected());
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().last_ipoc, 0u);
}

TEST(SessionMonitor, FirstFrameConnects) {
  RsiSessionMonitor m{SessionConfig{}};
  m.onFrame(frameWithIpoc(100));
  EXPECT_TRUE(m.connected());
  EXPECT_EQ(m.stats().last_ipoc, 100u);
  EXPECT_EQ(m.stats().ipoc_jumps, 0u);
}

TEST(SessionMonitor, TimeoutsBeforeConnectionNeverFault) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 3;
  RsiSessionMonitor m{cfg};
  for (int i = 0; i < 100; ++i) m.onTimeout();
  EXPECT_FALSE(m.faulted());
  EXPECT_FALSE(m.connected());
}

TEST(SessionMonitor, ConsecutiveTimeoutsLatchFault) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 3;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  m.onTimeout();
  EXPECT_FALSE(m.faulted());  // 2 < 3
  m.onTimeout();
  EXPECT_TRUE(m.faulted());  // 3rd consecutive miss
  EXPECT_EQ(m.stats().total_timeouts, 3u);
}

TEST(SessionMonitor, FrameResetsConsecutiveCount) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 3;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  m.onTimeout();
  m.onFrame(frameWithIpoc(2));
  EXPECT_EQ(m.stats().consecutive_timeouts, 0u);
  m.onTimeout();
  m.onTimeout();
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().total_timeouts, 4u);
}

TEST(SessionMonitor, BadFramesCountTowardFault) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 2;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onBadFrame();
  m.onBadFrame();
  EXPECT_TRUE(m.faulted());
  EXPECT_EQ(m.stats().bad_frames, 2u);
}

TEST(SessionMonitor, NonIncreasingIpocCountsJumpButNoFault) {
  RsiSessionMonitor m{SessionConfig{}};
  m.onFrame(frameWithIpoc(100));
  m.onFrame(frameWithIpoc(100));  // repeat
  m.onFrame(frameWithIpoc(50));   // backwards
  EXPECT_EQ(m.stats().ipoc_jumps, 2u);
  EXPECT_FALSE(m.faulted());
  EXPECT_EQ(m.stats().last_ipoc, 50u);  // tracks latest regardless
}

TEST(SessionMonitor, FaultIsLatchedUntilReset) {
  SessionConfig cfg;
  cfg.max_consecutive_timeouts = 1;
  RsiSessionMonitor m{cfg};
  m.onFrame(frameWithIpoc(1));
  m.onTimeout();
  ASSERT_TRUE(m.faulted());
  m.onFrame(frameWithIpoc(2));  // frames do NOT clear a latched fault
  EXPECT_TRUE(m.faulted());
  m.reset();
  EXPECT_FALSE(m.faulted());
  EXPECT_FALSE(m.connected());  // reset returns to initial state
  EXPECT_EQ(m.stats().total_timeouts, 0u);
}
