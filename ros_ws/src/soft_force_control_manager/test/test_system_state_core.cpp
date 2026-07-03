#include <gtest/gtest.h>

#include "soft_force_control_manager/system_state_core.h"

using sfm::HealthInputs;
using sfm::SystemState;
using sfm::SystemStateCore;

namespace {

HealthInputs offline() { return HealthInputs{}; }

HealthInputs connectedOnly() {
  HealthInputs in;
  in.eki_link = true;
  return in;
}

// Everything a READY verdict needs; RSI deliberately down (decision 2:
// the KRC does not stream RSI before START_RSI, READY must not need it).
HealthInputs ready() {
  HealthInputs in;
  in.eki_link = true;
  in.eki_program_ready = true;
  in.sri_streaming = true;
  in.tool_synced = true;
  in.controllers_loaded = true;
  return in;
}

HealthInputs full() {
  HealthInputs in = ready();
  in.rsi_topic_fresh = true;
  in.rsi_connected = true;
  return in;
}

}  // namespace

TEST(SystemState, IdleLadderFollowsHealth) {
  SystemStateCore c;
  EXPECT_EQ(c.update(offline()), SystemState::OFFLINE);
  EXPECT_EQ(c.update(connectedOnly()), SystemState::CONNECTED);
  EXPECT_EQ(c.update(ready()), SystemState::READY);   // no RSI needed
  EXPECT_EQ(c.update(connectedOnly()), SystemState::CONNECTED);  // back down
  EXPECT_EQ(c.update(offline()), SystemState::OFFLINE);
}

TEST(SystemState, StartRequiresFreshRsiTopic) {
  SystemStateCore c;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  EXPECT_FALSE(c.requestStart(ready()).accepted);  // rsi_topic_fresh false

  HealthInputs in = ready();
  in.rsi_topic_fresh = true;  // hw node alive; RSI frames come after START_RSI
  EXPECT_TRUE(c.requestStart(in).accepted);
  EXPECT_EQ(c.update(full()), SystemState::SERVOING);
}

TEST(SystemState, ServoDegradesAndRecovers) {
  SystemStateCore c;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);

  HealthInputs degraded = full();
  degraded.sri_streaming = false;
  EXPECT_EQ(c.update(degraded), SystemState::DEGRADED);
  EXPECT_EQ(c.update(full()), SystemState::SERVOING);  // auto-recovery

  EXPECT_TRUE(c.requestStop().accepted);
  EXPECT_EQ(c.update(full()), SystemState::READY);
}

TEST(SystemState, FaultLatchesUntilReset) {
  SystemStateCore c;
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);

  HealthInputs faulted = full();
  faulted.rsi_fault = true;
  EXPECT_EQ(c.update(faulted), SystemState::FAULT);
  EXPECT_EQ(c.update(full()), SystemState::FAULT);  // latched
  EXPECT_FALSE(c.requestStart(full()).accepted);
  EXPECT_FALSE(c.requestStop().accepted);

  EXPECT_TRUE(c.requestReset().accepted);
  // servo_requested_ was cleared by the fault: back to READY, not SERVOING.
  EXPECT_EQ(c.update(full()), SystemState::READY);
}

TEST(SystemState, CalibrationLifecycleAndLinkLoss) {
  SystemStateCore c;
  ASSERT_EQ(c.update(full()), SystemState::READY);
  EXPECT_FALSE(c.requestCalibration(ready()).accepted);  // rsi topic stale
  EXPECT_TRUE(c.requestCalibration(full()).accepted);
  EXPECT_EQ(c.update(full()), SystemState::CALIBRATING);

  c.calibrationFinished();
  EXPECT_EQ(c.update(full()), SystemState::READY);

  ASSERT_TRUE(c.requestCalibration(full()).accepted);
  ASSERT_EQ(c.update(full()), SystemState::CALIBRATING);
  HealthInputs lost = full();
  lost.sri_streaming = false;
  EXPECT_EQ(c.update(lost), SystemState::FAULT);  // fatal during calibration
  EXPECT_EQ(c.update(full()), SystemState::FAULT);
}

TEST(SystemState, CalibrationRejectedOutsideReady) {
  SystemStateCore c;
  EXPECT_FALSE(c.requestCalibration(full()).accepted);  // OFFLINE
  ASSERT_EQ(c.update(ready()), SystemState::READY);
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  ASSERT_TRUE(c.requestStart(start_in).accepted);
  ASSERT_EQ(c.update(full()), SystemState::SERVOING);
  EXPECT_FALSE(c.requestCalibration(full()).accepted);  // SERVOING
}

TEST(SystemState, ZeroSensorGateFollowsState) {
  SystemStateCore c;
  EXPECT_FALSE(c.allowZeroSensor());                    // OFFLINE
  c.update(connectedOnly());
  EXPECT_TRUE(c.allowZeroSensor());                     // CONNECTED
  c.update(ready());
  EXPECT_TRUE(c.allowZeroSensor());                     // READY
  HealthInputs start_in = ready();
  start_in.rsi_topic_fresh = true;
  c.requestStart(start_in);
  c.update(full());
  EXPECT_FALSE(c.allowZeroSensor());                    // SERVOING (dec. 11)
}

TEST(SystemState, RejectionsCarryReasons) {
  SystemStateCore c;
  const sfm::Verdict v = c.requestStart(offline());
  EXPECT_FALSE(v.accepted);
  EXPECT_STRNE(v.reason, "");
}
