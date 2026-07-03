#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "sri_force_torque_driver/sri_stream_session.h"
#include "sri_force_torque_driver/tcp_client_transport.h"

namespace sri {

struct SriDriverConfig {
  std::string sensor_ip{"127.0.0.1"};
  std::uint16_t sensor_port{4008};
  int connect_timeout_ms{1000};
  int receive_timeout_ms{50};
  double reconnect_backoff_s{0.5};
  SriSessionConfig session;
};

struct SriDriverStatus {
  bool connected{false};
  std::uint32_t reconnects{0};  // completed re-connections after the first
  SriStatusSnapshot session;
};

// Owns the rx thread: bounded connect -> AT+GSD -> receive/parse loop with
// automatic reconnect. The sample callback runs synchronously in the rx
// thread right after the socket read, so its invocation time IS the
// reception instant (the node stamps messages there; Plan 3 follow-up 3).
// Not ROS-dependent. Service-facing methods (requestZero/setFilterCutoff/
// status) synchronize with the rx thread through mutex_.
class SriDriverRuntime {
 public:
  using SampleCallback = std::function<void(const SriWrenchSample&)>;

  explicit SriDriverRuntime(const SriDriverConfig& cfg);
  ~SriDriverRuntime();
  SriDriverRuntime(const SriDriverRuntime&) = delete;
  SriDriverRuntime& operator=(const SriDriverRuntime&) = delete;

  void setSampleCallback(SampleCallback cb);  // call before start()
  bool start();
  void stop();

  // Starts a tare capture and waits (bounded) for it to finish. false on
  // timeout (capture cancelled, old bias kept) or rejected bias limit.
  bool requestZero(int timeout_ms);
  bool setFilterCutoff(double cutoff_hz);  // false for negative values
  SriDriverStatus status() const;

 private:
  void run();
  static double nowS();

  SriDriverConfig cfg_;
  SampleCallback callback_;
  TcpClientTransport transport_;  // rx thread only
  std::thread thread_;
  std::atomic<bool> running_{false};
  mutable std::mutex mutex_;  // guards session_, connected_, reconnects_
  SriStreamSession session_;
  bool connected_{false};
  bool was_connected_once_{false};
  std::uint32_t reconnects_{0};
};

}  // namespace sri
