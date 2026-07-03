#include "sri_force_torque_driver/sri_mock_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>

namespace sri {

namespace {
constexpr std::size_t kFrameLen = 31;  // AA 55 LL LL PN PN 24xdata SUM
constexpr int kPollMs = 10;

bool waitFlag(const std::atomic<bool>& flag, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (flag.load()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return flag.load();
}
}  // namespace

SriMockServer::SriMockServer(const SriMockConfig& cfg) : cfg_(cfg) {}

SriMockServer::~SriMockServer() { stop(); }

bool SriMockServer::start() {
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) return false;
  int on = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::inet_addr("127.0.0.1");
  addr.sin_port = htons(cfg_.listen_port);
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) !=
          0 ||
      ::listen(listen_fd_, 1) != 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  socklen_t len = sizeof(addr);
  ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  running_ = true;
  thread_ = std::thread(&SriMockServer::run, this);
  return true;
}

void SriMockServer::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  dropClient();
  if (listen_fd_ >= 0) ::close(listen_fd_);
  listen_fd_ = -1;
}

bool SriMockServer::waitForClient(int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (client_fd_ >= 0) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return client_fd_ >= 0;
}

bool SriMockServer::waitForStartCommand(int timeout_ms) {
  return waitFlag(streaming_, timeout_ms);
}

void SriMockServer::setWrench(float fx, float fy, float fz, float mx,
                              float my, float mz) {
  std::lock_guard<std::mutex> lock(mutex_);
  wrench_[0] = fx;
  wrench_[1] = fy;
  wrench_[2] = fz;
  wrench_[3] = mx;
  wrench_[4] = my;
  wrench_[5] = mz;
}

std::size_t SriMockServer::buildFrame(std::uint8_t* buf,
                                      bool corrupt_checksum) {
  // Caller holds mutex_ (wrench_/pn_ access).
  buf[0] = 0xAA;
  buf[1] = 0x55;
  buf[2] = 0x00;
  buf[3] = 0x1B;  // payload length 27 = PN(2) + data(24) + sum(1)
  ++pn_;
  buf[4] = static_cast<std::uint8_t>(pn_ >> 8);
  buf[5] = static_cast<std::uint8_t>(pn_ & 0xFF);
  unsigned sum = 0;
  for (int i = 0; i < 6; ++i) {
    std::memcpy(buf + 6 + 4 * i, &wrench_[i], 4);  // little-endian host
  }
  for (int k = 6; k < 30; ++k) sum += buf[k];
  buf[30] = static_cast<std::uint8_t>(sum & 0xFF);
  if (corrupt_checksum) buf[30] = static_cast<std::uint8_t>(buf[30] ^ 0xFF);
  return kFrameLen;
}

void SriMockServer::sendLocked(const void* data, std::size_t len) {
  // Caller holds mutex_.
  if (client_fd_ < 0) return;
#ifdef MSG_NOSIGNAL
  ::send(client_fd_, data, len, MSG_NOSIGNAL);
#else
  ::send(client_fd_, data, len, 0);
#endif
}

void SriMockServer::sendFrames(int count) {
  if (!streaming_.load() && cfg_.require_start_command) return;
  std::lock_guard<std::mutex> lock(mutex_);
  std::uint8_t buf[kFrameLen];
  for (int i = 0; i < count; ++i) {
    buildFrame(buf, false);
    sendLocked(buf, kFrameLen);
    ++frames_sent_;
  }
}

void SriMockServer::sendBadChecksumFrame() {
  if (!streaming_.load() && cfg_.require_start_command) return;
  std::lock_guard<std::mutex> lock(mutex_);
  std::uint8_t buf[kFrameLen];
  buildFrame(buf, true);
  sendLocked(buf, kFrameLen);
}

void SriMockServer::sendRaw(const void* data, std::size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  sendLocked(data, len);
}

void SriMockServer::dropClient() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (client_fd_ >= 0) {
    ::shutdown(client_fd_, SHUT_RDWR);
    ::close(client_fd_);
    client_fd_ = -1;
  }
  streaming_ = false;
}

void SriMockServer::run() {
  using clock = std::chrono::steady_clock;
  auto next_paced = clock::now();
  while (running_.load()) {
    int client;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      client = client_fd_;
    }
    if (client < 0) {
      pollfd p{listen_fd_, POLLIN, 0};
      if (::poll(&p, 1, kPollMs) > 0) {
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd >= 0) {
          std::lock_guard<std::mutex> lock(mutex_);
          client_fd_ = fd;
          streaming_ = !cfg_.require_start_command;
        }
      }
      continue;
    }
    pollfd p{client, POLLIN, 0};
    if (::poll(&p, 1, kPollMs) > 0) {
      char buf[256];
      const ssize_t n = ::recv(client, buf, sizeof(buf), 0);
      if (n <= 0) {
        dropClient();
        continue;
      }
      static const char kCmd[] = "AT+GSD";
      if (std::search(buf, buf + n, kCmd, kCmd + 6) != buf + n) {
        static const char kAck[] = "ACK+GSD=OK\r\n";
        std::lock_guard<std::mutex> lock(mutex_);
        sendLocked(kAck, sizeof(kAck) - 1);
        streaming_ = true;
      }
    }
    if (cfg_.rate_hz > 0.0 && streaming_.load() &&
        clock::now() >= next_paced) {
      std::lock_guard<std::mutex> lock(mutex_);
      std::uint8_t frame[kFrameLen];
      buildFrame(frame, false);
      sendLocked(frame, kFrameLen);
      ++frames_sent_;
      next_paced = clock::now() + std::chrono::microseconds(
                                      static_cast<long>(1e6 / cfg_.rate_hz));
    }
  }
}

}  // namespace sri
