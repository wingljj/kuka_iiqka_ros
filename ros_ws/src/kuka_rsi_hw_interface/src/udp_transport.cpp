#include "kuka_rsi_hw_interface/udp_transport.h"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace kuka_rsi {

bool UdpTransport::bind(const std::string& ip, std::uint16_t port) {
  close();
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) return false;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    close();
    return false;
  }
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close();
    return false;
  }
  sockaddr_in bound{};
  socklen_t len = sizeof(bound);
  if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
    close();
    return false;
  }
  bound_port_ = ntohs(bound.sin_port);
  return true;
}

int UdpTransport::receive(char* buf, std::size_t buf_size, int timeout_ms) {
  if (fd_ < 0) return -1;
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr == 0) return 0;   // timeout
  if (pr < 0) return -1;   // error
  sockaddr_in sender{};
  socklen_t sender_len = sizeof(sender);
  const ssize_t n =
      ::recvfrom(fd_, buf, buf_size, 0,
                 reinterpret_cast<sockaddr*>(&sender), &sender_len);
  if (n < 0) return -1;
  last_sender_ = sender;
  has_last_sender_ = true;
  return static_cast<int>(n);
}

bool UdpTransport::sendToLastSender(const char* data, std::size_t len) {
  if (fd_ < 0 || !has_last_sender_) return false;
  const ssize_t n =
      ::sendto(fd_, data, len, 0,
               reinterpret_cast<const sockaddr*>(&last_sender_),
               sizeof(last_sender_));
  return n == static_cast<ssize_t>(len);
}

bool UdpTransport::sendTo(const std::string& ip, std::uint16_t port,
                          const char* data, std::size_t len) {
  if (fd_ < 0) return false;
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &dst.sin_addr) != 1) return false;
  const ssize_t n = ::sendto(fd_, data, len, 0,
                             reinterpret_cast<const sockaddr*>(&dst),
                             sizeof(dst));
  return n == static_cast<ssize_t>(len);
}

void UdpTransport::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  bound_port_ = 0;
  has_last_sender_ = false;
}

}  // namespace kuka_rsi
