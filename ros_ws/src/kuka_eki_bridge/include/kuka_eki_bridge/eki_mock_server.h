#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_eki_bridge/eki_frame.h"

namespace kuka_eki {

struct EkiMockConfig {
  double heartbeat_period_s{0.0};  // 0 = heartbeats only via pushHeartbeat()
};

// KRC-side EKI mock (spec 15.2): TCP server on 127.0.0.1 (kernel-chosen
// port), single client. Parses RobotCommand documents and answers with
// RobotState acks according to a small behavior table mirroring the Plan 5
// KRL program (ready/start/stop/fault/reset/set-mode/tool). Test code:
// threads/locks are fine here (same stance as kuka_rsi::RsiMockServer).
class EkiMockServer {
 public:
  explicit EkiMockServer(const EkiMockConfig& cfg);
  ~EkiMockServer();
  EkiMockServer(const EkiMockServer&) = delete;
  EkiMockServer& operator=(const EkiMockServer&) = delete;

  bool start();
  void stop();
  std::uint16_t port() const { return port_; }

  bool waitForClient(int timeout_ms);
  void dropClient();

  // Scripting surface.
  void pushHeartbeat();                 // one unsolicited RobotState frame
  void setReady(bool ready);
  void injectFault();
  void setTool(double x, double y, double z, double a, double b, double c);
  void setRespondToNext(bool respond);  // false: swallow one command
  void sendMalformed();

  bool rsiActive() const;
  bool fault() const;
  int mode() const;
  std::uint64_t commandsReceived() const { return commands_.load(); }

 private:
  void run();
  void handleCommand(const EkiCommand& cmd);
  void sendStateLocked(std::uint32_t ack_seq, bool ok, int code);
  bool parseCommand(const char* data, std::size_t len, EkiCommand& out);

  EkiMockConfig cfg_;
  int listen_fd_{-1};
  std::uint16_t port_{0};
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<std::uint64_t> commands_{0};
  mutable std::mutex mutex_;  // guards client_fd_ and the state table
  int client_fd_{-1};
  std::string rx_buffer_;
  bool ready_{true};
  bool rsi_active_{false};
  bool fault_{false};
  int mode_{0};
  Frame6 tool_;
  bool respond_next_{true};
};

}  // namespace kuka_eki
