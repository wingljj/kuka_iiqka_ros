#include "kuka_eki_bridge/eki_session_core.h"

namespace kuka_eki {

CommandOutcome EkiSessionCore::reset() {
  const bool had_pending = pending_;
  pending_ = false;
  last_state_s_ = -1.0;
  last_state_ = EkiStateFrame{};
  if (had_pending) {
    ++timeouts_;
    return CommandOutcome::TIMEOUT;
  }
  return CommandOutcome::NONE;
}

bool EkiSessionCore::beginCommand(std::uint32_t seq, double now_s) {
  if (pending_) return false;
  pending_ = true;
  pending_seq_ = seq;
  pending_since_s_ = now_s;
  return true;
}

CommandOutcome EkiSessionCore::onState(const EkiStateFrame& frame,
                                       double now_s) {
  ++states_;
  last_state_ = frame;
  last_state_s_ = now_s;
  if (pending_ && frame.ack_seq == pending_seq_ && frame.ack_seq != 0) {
    pending_ = false;
    return frame.ack_ok ? CommandOutcome::ACCEPTED : CommandOutcome::REJECTED;
  }
  return CommandOutcome::NONE;
}

CommandOutcome EkiSessionCore::tick(double now_s) {
  if (pending_ && now_s - pending_since_s_ > cfg_.response_timeout_s) {
    pending_ = false;
    ++timeouts_;
    return CommandOutcome::TIMEOUT;
  }
  return CommandOutcome::NONE;
}

EkiSessionSnapshot EkiSessionCore::snapshot(double now_s) const {
  EkiSessionSnapshot s;
  s.last_state = last_state_;
  s.states = states_;
  s.bad_frames = bad_frames_;
  s.timeouts = timeouts_;
  if (last_state_s_ >= 0.0) {
    s.state_age_s = now_s - last_state_s_;
    s.state_fresh = s.state_age_s <= cfg_.state_timeout_s;
  }
  return s;
}

}  // namespace kuka_eki
