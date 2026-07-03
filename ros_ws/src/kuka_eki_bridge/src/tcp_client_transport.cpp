#include "kuka_eki_bridge/tcp_client_transport.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>

namespace kuka_eki {

namespace {
int remainingMs(const std::chrono::steady_clock::time_point& deadline) {
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
  return left.count() > 0 ? static_cast<int>(left.count()) : 0;
}
}  // namespace

bool TcpClientTransport::connect(const std::string& ip, std::uint16_t port,
                                 int timeout_ms) {
  close();
  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) return false;
  ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  int on = 1;
  ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    close();
    return false;
  }
  const int rc =
      ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (rc != 0 && errno != EINPROGRESS) {
    close();
    return false;
  }
  if (rc != 0) {
    pollfd p{fd_, POLLOUT, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) {
      close();
      return false;
    }
    int err = 0;
    socklen_t len = sizeof(err);
    if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
      close();
      return false;
    }
  }
  return true;
}

int TcpClientTransport::receive(char* buf, std::size_t buf_size,
                                int timeout_ms) {
  if (fd_ < 0) return -1;
  pollfd p{fd_, POLLIN, 0};
  const int rc = ::poll(&p, 1, timeout_ms);
  if (rc == 0) return 0;
  if (rc < 0) {
    close();
    return -1;
  }
  const ssize_t n = ::recv(fd_, buf, buf_size, 0);
  if (n > 0) return static_cast<int>(n);
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
  close();  // n == 0: orderly peer close; n < 0: hard error
  return -1;
}

bool TcpClientTransport::send(const char* data, std::size_t len,
                              int timeout_ms) {
  if (fd_ < 0) return false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  std::size_t sent = 0;
  while (sent < len) {
    pollfd p{fd_, POLLOUT, 0};
    if (::poll(&p, 1, remainingMs(deadline)) <= 0) {
      close();
      return false;
    }
#ifdef MSG_NOSIGNAL
    const ssize_t n = ::send(fd_, data + sent, len - sent, MSG_NOSIGNAL);
#else
    const ssize_t n = ::send(fd_, data + sent, len - sent, 0);
#endif
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    if (n <= 0) {
      close();
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

void TcpClientTransport::close() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

}  // namespace kuka_eki
