#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "sri_force_torque_driver/tcp_client_transport.h"

using sri::TcpClientTransport;

namespace {

// Minimal loopback listener used to play the sensor-box side.
class TestListener {
 public:
  bool start() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0) return false;
    int on = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
    addr.sin_port = 0;  // kernel-chosen port
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
      return false;
    socklen_t len = sizeof(addr);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    return ::listen(fd_, 1) == 0;
  }
  bool acceptClient(int timeout_ms) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return false;
    client_ = ::accept(fd_, nullptr, nullptr);
    return client_ >= 0;
  }
  int readClient(char* buf, std::size_t n, int timeout_ms) {
    pollfd p{client_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return 0;
    return static_cast<int>(::recv(client_, buf, n, 0));
  }
  bool writeClient(const char* data, std::size_t n) {
    return ::send(client_, data, n, 0) == static_cast<ssize_t>(n);
  }
  void closeClient() {
    if (client_ >= 0) ::close(client_);
    client_ = -1;
  }
  void stop() {
    closeClient();
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  ~TestListener() { stop(); }
  std::uint16_t port() const { return port_; }

 private:
  int fd_{-1};
  int client_{-1};
  std::uint16_t port_{0};
};

}  // namespace

TEST(TcpTransport, ConnectRefusedFails) {
  TestListener l;
  ASSERT_TRUE(l.start());
  const std::uint16_t dead_port = l.port();
  l.stop();  // nobody listens there anymore
  TcpClientTransport t;
  EXPECT_FALSE(t.connect("127.0.0.1", dead_port, 200));
  EXPECT_FALSE(t.connected());
}

TEST(TcpTransport, ConnectAndExchange) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  ASSERT_TRUE(t.send("ping", 4, 200));
  char buf[16] = {0};
  ASSERT_EQ(l.readClient(buf, sizeof(buf), 500), 4);
  EXPECT_EQ(std::string(buf, 4), "ping");
  ASSERT_TRUE(l.writeClient("pong!", 5));
  ASSERT_EQ(t.receive(buf, sizeof(buf), 500), 5);
  EXPECT_EQ(std::string(buf, 5), "pong!");
}

TEST(TcpTransport, ReceiveTimeoutReturnsZero) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 50), 0);
  EXPECT_TRUE(t.connected());
}

TEST(TcpTransport, PeerCloseReturnsMinusOne) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  l.closeClient();
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 500), -1);
  EXPECT_FALSE(t.connected());
}

TEST(TcpTransport, SendWithoutConnectFails) {
  TcpClientTransport t;
  EXPECT_FALSE(t.send("x", 1, 100));
}

TEST(TcpTransport, ReconnectAfterClose) {
  TestListener l;
  ASSERT_TRUE(l.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  t.close();
  EXPECT_FALSE(t.connected());
  l.closeClient();
  ASSERT_TRUE(t.connect("127.0.0.1", l.port(), 500));
  ASSERT_TRUE(l.acceptClient(500));
  EXPECT_TRUE(t.connected());
}
