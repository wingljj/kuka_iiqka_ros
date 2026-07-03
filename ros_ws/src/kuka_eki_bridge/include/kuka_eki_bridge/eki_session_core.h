#pragma once

#include <cstdint>

#include "kuka_eki_bridge/eki_frame.h"

namespace kuka_eki {

struct EkiSessionConfig {
  double response_timeout_s{2.0};  // ack deadline for one command
  double state_timeout_s{1.0};     // heartbeat freshness threshold
};

enum class CommandOutcome { NONE, ACCEPTED, REJECTED, TIMEOUT };

struct EkiSessionSnapshot {
  bool state_fresh{false};
  double state_age_s{-1.0};  // -1 until the first state frame
  EkiStateFrame last_state;
  std::uint64_t states{0};
  std::uint64_t bad_frames{0};
  std::uint64_t timeouts{0};  // command response timeouts (incl. resets)
};

// Request/response bookkeeping of the management channel (decision 7):
// one in-flight command at a time, correlated by Ack.Seq; every state
// frame (ack or heartbeat) refreshes freshness and last_state. Pure
// logic, single-threaded (the runtime's io thread serializes access).
class EkiSessionCore {
 public:
  void configure(const EkiSessionConfig& cfg) { cfg_ = cfg; }

  // Disconnect hook: fails a pending command with TIMEOUT (returned so
  // the runtime can resolve its waiter) and clears freshness.
  CommandOutcome reset();

  // Registers command `seq` as in-flight. false if one is pending.
  bool beginCommand(std::uint32_t seq, double now_s);
  bool commandPending() const { return pending_; }

  // Processes one parsed state frame. ACCEPTED/REJECTED when it acks the
  // pending command; NONE for heartbeats and stale acks.
  CommandOutcome onState(const EkiStateFrame& frame, double now_s);
  void onBadFrame() { ++bad_frames_; }

  // Periodic timeout check for the pending command.
  CommandOutcome tick(double now_s);

  EkiSessionSnapshot snapshot(double now_s) const;

 private:
  EkiSessionConfig cfg_;
  bool pending_{false};
  std::uint32_t pending_seq_{0};
  double pending_since_s_{0};
  EkiStateFrame last_state_;
  double last_state_s_{-1.0};
  std::uint64_t states_{0};
  std::uint64_t bad_frames_{0};
  std::uint64_t timeouts_{0};
};

}  // namespace kuka_eki
