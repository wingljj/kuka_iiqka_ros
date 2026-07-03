#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace kuka_rsi {

// One reply window with stale-backlog resync (Task 8c debt fix): keeps
// receiving until a reply matching the awaited IPOC is applied (returns
// true) or timeout_ms expires (returns false). Stale replies left queued
// by a PC-side stall are consumed inside the same window -- each still
// bumps ipoc_echo_errors via applyReply -- so the mock recovers by itself
// instead of staying one packet behind forever. If nothing at all arrives
// in the window, noteReplyTimeout() is recorded. `mutex`, when non-null,
// is held around every core access so a threaded caller can read stats
// concurrently. Test/mock code: not realtime.
bool receiveReplyWindow(RsiMockCore& core, UdpTransport& udp,
                        int timeout_ms, std::mutex* mutex = nullptr);

// Threaded UDP wrapper around RsiMockCore playing the KRC role: the mock
// is the UDP client, it sends the state frame first and waits for the
// PC's <Sen> reply each cycle (spec section 6.1). Test code: locking and
// threads are fine here, this never runs in the realtime path. Cycles run
// back-to-back (no 4 ms pacing) so tests finish fast; the standalone
// kuka_rsi_sim_server adds real pacing via --cycle-ms.
class RsiMockServer {
 public:
  RsiMockServer(const MockConfig& cfg, const std::string& target_ip,
                std::uint16_t target_port, int reply_timeout_ms = 50);
  ~RsiMockServer() { stop(); }

  bool start();
  void stop();

  MockStats statsSnapshot() const;
  void setPose(double x, double y, double z, double a, double b, double c);
  double poseX() const;

 private:
  void run();

  RsiMockCore core_;
  std::string target_ip_;
  std::uint16_t target_port_;
  int reply_timeout_ms_;
  UdpTransport udp_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex mutex_;
};

}  // namespace kuka_rsi
