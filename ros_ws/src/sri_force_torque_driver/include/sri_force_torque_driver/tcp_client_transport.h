#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sri {

// Minimal bounded-wait TCP client, structural twin of the UDP endpoint
// from kuka_rsi_hw_interface (poll()-based waits, no allocation after
// connect, not copyable). Any receive error / peer close / send failure
// closes the socket so connected() doubles as the reconnect trigger for
// the owning runtime. Not thread-safe: one owning thread at a time.
class TcpClientTransport {
 public:
  TcpClientTransport() = default;
  ~TcpClientTransport() { close(); }
  TcpClientTransport(const TcpClientTransport&) = delete;
  TcpClientTransport& operator=(const TcpClientTransport&) = delete;

  // Non-blocking connect with a bounded wait. false on refusal/timeout.
  bool connect(const std::string& ip, std::uint16_t port, int timeout_ms);
  bool connected() const { return fd_ >= 0; }

  // Waits up to timeout_ms. Returns byte count, 0 on timeout, -1 when the
  // peer closed or the socket errored (the transport closes itself).
  int receive(char* buf, std::size_t buf_size, int timeout_ms);

  // Sends the whole buffer within timeout_ms overall. false (and close)
  // on error or timeout.
  bool send(const char* data, std::size_t len, int timeout_ms);

  void close();

 private:
  int fd_{-1};
};

}  // namespace sri
