#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace sri {

struct SriMockConfig {
  bool require_start_command{true};  // hold frames until AT+GSD arrives
  double rate_hz{0.0};               // 0 = frames only via sendFrames()
  std::uint16_t listen_port{0};      // 0 = kernel-chosen (default; tests)
};

// Scriptable stand-in for the SRI acquisition box: TCP server on
// 127.0.0.1 (kernel-chosen port), single client, answers AT+GSD with an
// ASCII ACK line, then emits binary frames on demand (scripted) or paced
// (rate_hz > 0). Test code: threads/locks are fine here, this never runs
// in a realtime path (same stance as kuka_rsi::RsiMockServer).
class SriMockServer {
 public:
  explicit SriMockServer(const SriMockConfig& cfg);
  ~SriMockServer();
  SriMockServer(const SriMockServer&) = delete;
  SriMockServer& operator=(const SriMockServer&) = delete;

  bool start();
  void stop();
  std::uint16_t port() const { return port_; }

  bool waitForClient(int timeout_ms);
  bool waitForStartCommand(int timeout_ms);

  void setWrench(float fx, float fy, float fz, float mx, float my, float mz);
  void sendFrames(int count);       // ignored while not streaming
  void sendBadChecksumFrame();
  void sendRaw(const void* data, std::size_t len);
  void dropClient();
  std::uint64_t framesSent() const { return frames_sent_.load(); }

 private:
  void run();
  void sendLocked(const void* data, std::size_t len);
  std::size_t buildFrame(std::uint8_t* buf, bool corrupt_checksum);

  SriMockConfig cfg_;
  int listen_fd_{-1};
  std::uint16_t port_{0};
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<std::uint64_t> frames_sent_{0};
  mutable std::mutex mutex_;  // guards client_fd_, wrench_, pn_
  int client_fd_{-1};
  float wrench_[6] = {0, 0, 0, 0, 0, 0};
  std::uint16_t pn_{0};
};

}  // namespace sri
