#include "kuka_rsi_hw_interface/rsi_session_monitor.h"

namespace kuka_rsi {

void RsiSessionMonitor::onFrame(const RobFrame& frame) {
  if (stats_.connected && frame.ipoc <= stats_.last_ipoc) {
    ++stats_.ipoc_jumps;
  }
  stats_.last_ipoc = frame.ipoc;
  stats_.connected = true;
  stats_.consecutive_timeouts = 0;
}

void RsiSessionMonitor::onTimeout() {
  if (!stats_.connected) return;  // KRC not started yet: benign
  ++stats_.total_timeouts;
  countMiss();
}

void RsiSessionMonitor::onBadFrame() {
  if (!stats_.connected) return;
  ++stats_.bad_frames;
  countMiss();
}

void RsiSessionMonitor::countMiss() {
  ++stats_.consecutive_timeouts;
  if (stats_.consecutive_timeouts >= cfg_.max_consecutive_timeouts) {
    stats_.fault = true;  // latched until clearFault()/reset()
  }
}

void RsiSessionMonitor::reset() { stats_ = SessionStats{}; }

void RsiSessionMonitor::clearFault() {
  stats_.fault = false;
  stats_.consecutive_timeouts = 0;
}

}  // namespace kuka_rsi
