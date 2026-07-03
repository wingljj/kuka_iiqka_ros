#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace kuka_rsi {

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
