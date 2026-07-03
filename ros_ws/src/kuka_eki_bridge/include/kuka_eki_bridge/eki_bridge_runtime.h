#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "kuka_eki_bridge/eki_frame.h"
#include "kuka_eki_bridge/eki_session_core.h"
#include "kuka_eki_bridge/eki_stream_splitter.h"
#include "kuka_eki_bridge/tcp_client_transport.h"

namespace kuka_eki {

// Bridge-local error code; KRC-side Ack codes are non-negative.
constexpr int kErrNotConnected = -1;

struct EkiBridgeConfig {
  std::string kuka_ip{"127.0.0.1"};
  std::uint16_t eki_port{54600};
  int connect_timeout_ms{1000};
  int receive_timeout_ms{50};
  double reconnect_backoff_s{1.0};
  bool auto_reconnect{true};
  EkiSessionConfig session;
};

struct ExecuteResult {
  CommandOutcome outcome{CommandOutcome::NONE};
  int error_code{0};   // Ack.Code on REJECTED; kErrNotConnected; else 0
  EkiStateFrame state; // latest state frame at resolution time
};

struct EkiBridgeStatus {
  bool connected{false};
  std::uint32_t reconnects{0};
  EkiSessionSnapshot session;
};

// Owns the io thread (connect/reconnect, receive -> splitter -> parser ->
// session, timeout ticks) and a blocking execute() used by the service
// layer (decision 12). The io thread is the only toucher of the socket;
// execute() posts a request into a shared slot and waits on a condition
// variable for the io thread's terminal verdict — never PENDING, bounded
// by response_timeout_s + 1 s. Not ROS-dependent.
class EkiBridgeRuntime {
 public:
  explicit EkiBridgeRuntime(const EkiBridgeConfig& cfg);
  ~EkiBridgeRuntime();
  EkiBridgeRuntime(const EkiBridgeRuntime&) = delete;
  EkiBridgeRuntime& operator=(const EkiBridgeRuntime&) = delete;

  bool start();
  void stop();

  // Waits (bounded) until the link is up. With auto_reconnect the io
  // thread connects on its own; this only observes.
  bool connectNow(int timeout_ms);

  ExecuteResult execute(EkiAction action, int value = 0,
                        const Frame6& tool = Frame6{},
                        const Frame6& base = Frame6{});

  EkiBridgeStatus status() const;

 private:
  void run();
  void resolveLocked(CommandOutcome outcome, int error_code);
  static double nowS();

  EkiBridgeConfig cfg_;
  TcpClientTransport transport_;  // io thread only
  EkiStreamSplitter splitter_;    // io thread only
  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex command_mutex_;  // serializes execute() callers

  mutable std::mutex mutex_;  // guards everything below
  std::condition_variable cv_;
  EkiSessionCore session_;
  bool connected_{false};
  bool was_connected_once_{false};
  std::uint32_t reconnects_{0};
  bool connect_requested_{false};
  std::uint32_t next_seq_{1};
  bool req_active_{false};
  EkiCommand req_cmd_;
  bool result_ready_{false};
  ExecuteResult result_;
};

}  // namespace kuka_eki
