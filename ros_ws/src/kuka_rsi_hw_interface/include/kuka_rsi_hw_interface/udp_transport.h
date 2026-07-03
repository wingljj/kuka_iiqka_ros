#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace kuka_rsi {

// Minimal non-blocking UDP endpoint shared by the PC-side hardware
// interface and the KRC-side mock. receive() uses poll() with an explicit
// timeout so the realtime read path has a bounded wait. No allocation or
// locking after bind(). Not copyable.
class UdpTransport {
 public:
  UdpTransport() = default;
  ~UdpTransport() { close(); }
  UdpTransport(const UdpTransport&) = delete;
  UdpTransport& operator=(const UdpTransport&) = delete;

  // Binds the local endpoint. port 0 lets the kernel pick a free port
  // (used by tests to avoid collisions). Non-realtime; call during init.
  bool bind(const std::string& ip, std::uint16_t port);
  std::uint16_t boundPort() const { return bound_port_; }

  // Waits up to timeout_ms for a datagram. Returns byte count, 0 on
  // timeout, -1 on error/unbound. Remembers the sender for replies.
  int receive(char* buf, std::size_t buf_size, int timeout_ms);

  // Sends to the peer that sent the last received datagram (RSI role: the
  // KRC is the UDP client; the PC only ever answers).
  bool sendToLastSender(const char* data, std::size_t len);

  // Sends to an explicit destination (used by the mock to start the cycle).
  bool sendTo(const std::string& ip, std::uint16_t port, const char* data,
              std::size_t len);

  void close();

 private:
  int fd_{-1};
  std::uint16_t bound_port_{0};
  sockaddr_in last_sender_{};
  bool has_last_sender_{false};
};

}  // namespace kuka_rsi
