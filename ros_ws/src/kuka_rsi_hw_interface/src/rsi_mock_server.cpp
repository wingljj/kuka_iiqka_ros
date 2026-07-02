#include "kuka_rsi_hw_interface/rsi_mock_server.h"

namespace kuka_rsi {

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
  char rx[1024];
  while (running_) {
    std::size_t n;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      n = core_.buildStateFrame(tx, sizeof(tx));
    }
    if (n == 0) break;
    if (!udp_.sendTo(target_ip_, target_port_, tx, n)) break;
    const int r = udp_.receive(rx, sizeof(rx), reply_timeout_ms_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (r > 0) {
      core_.applyReply(rx, static_cast<std::size_t>(r));
    } else {
      core_.noteReplyTimeout();
    }
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
