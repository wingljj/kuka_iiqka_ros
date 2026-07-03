#pragma once

#include <cstdint>

#include "kuka_rsi_hw_interface/rsi_frame.h"

namespace kuka_rsi {

struct SessionConfig {
  // Consecutive missed/bad cycles after connection before latching a fault.
  unsigned max_consecutive_timeouts{5};
};

struct SessionStats {
  bool connected{false};
  bool fault{false};
  std::uint64_t last_ipoc{0};
  std::uint64_t total_timeouts{0};
  unsigned consecutive_timeouts{0};
  std::uint64_t bad_frames{0};
  std::uint64_t ipoc_jumps{0};  // non-increasing IPOC events (diagnostic only)
};

// Tracks RSI link health (spec section 12.2): IPOC continuity, timeout
// runs, and a latched communication fault. Timeouts before the first valid
// frame never fault (the KRC simply has not started RSI yet). A latched
// fault is only cleared by reset() — the manager/EKI layer owns recovery.
// All methods are allocation-free and non-blocking.
class RsiSessionMonitor {
 public:
  explicit RsiSessionMonitor(const SessionConfig& cfg) : cfg_(cfg) {}

  void onFrame(const RobFrame& frame);
  void onTimeout();
  void onBadFrame();
  void reset();

  bool connected() const { return stats_.connected; }
  bool faulted() const { return stats_.fault; }
  const SessionStats& stats() const { return stats_; }

 private:
  void countMiss();
  SessionConfig cfg_;
  SessionStats stats_;
};

}  // namespace kuka_rsi
