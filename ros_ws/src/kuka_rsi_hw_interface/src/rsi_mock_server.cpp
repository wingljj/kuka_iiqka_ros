#include "kuka_rsi_hw_interface/rsi_mock_server.h"

#include <chrono>

namespace kuka_rsi {

bool receiveReplyWindow(RsiMockCore& core, UdpTransport& udp,
                        int timeout_ms, std::mutex* mutex) {
  char rx[1024];
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  bool got_reply = false;
  for (;;) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) break;
    const int r = udp.receive(rx, sizeof(rx), static_cast<int>(remaining.count()));
    if (r <= 0) break;
    got_reply = true;
    bool applied;
    if (mutex != nullptr) {
      std::lock_guard<std::mutex> lock(*mutex);
      applied = core.applyReply(rx, static_cast<std::size_t>(r));
    } else {
      applied = core.applyReply(rx, static_cast<std::size_t>(r));
    }
    if (applied) return true;
    // Stale/garbage packet (already counted by applyReply): keep draining
    // the backlog until the fresh echo shows up or the window expires.
  }
  if (!got_reply) {
    if (mutex != nullptr) {
      std::lock_guard<std::mutex> lock(*mutex);
      core.noteReplyTimeout();
    } else {
      core.noteReplyTimeout();
    }
  }
  return false;
}

RsiMockServer::RsiMockServer(const MockConfig& cfg,
                             const std::string& target_ip,
                             std::uint16_t target_port, int reply_timeout_ms)
    : core_(cfg),
      target_ip_(target_ip),
      target_port_(target_port),
      reply_timeout_ms_(reply_timeout_ms) {}

bool RsiMockServer::start() {
  if (running_) return false;
  if (!udp_.bind("127.0.0.1", 0)) return false;
  running_ = true;
  thread_ = std::thread(&RsiMockServer::run, this);
  return true;
}

void RsiMockServer::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  udp_.close();
}

void RsiMockServer::run() {
  char tx[1024];
  while (running_) {
    std::size_t n;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      n = core_.buildStateFrame(tx, sizeof(tx));
    }
    if (n == 0) break;
    if (!udp_.sendTo(target_ip_, target_port_, tx, n)) break;
    receiveReplyWindow(core_, udp_, reply_timeout_ms_, &mutex_);
  }
}

MockStats RsiMockServer::statsSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return core_.stats();
}

void RsiMockServer::setPose(double x, double y, double z, double a, double b,
                            double c) {
  std::lock_guard<std::mutex> lock(mutex_);
  core_.setPose(x, y, z, a, b, c);
}

double RsiMockServer::poseX() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return core_.x();
}

}  // namespace kuka_rsi
