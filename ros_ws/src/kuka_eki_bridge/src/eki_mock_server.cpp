#include "kuka_eki_bridge/eki_mock_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <tinyxml2.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>

namespace kuka_eki {

namespace {
constexpr int kPollMs = 10;
constexpr const char kCmdEndTag[] = "</RobotCommand>";
}  // namespace

EkiMockServer::EkiMockServer(const EkiMockConfig& cfg) : cfg_(cfg) {}

EkiMockServer::~EkiMockServer() { stop(); }

bool EkiMockServer::start() {
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
  thread_ = std::thread(&EkiMockServer::run, this);
  return true;
}

void EkiMockServer::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  dropClient();
  if (listen_fd_ >= 0) ::close(listen_fd_);
  listen_fd_ = -1;
}

bool EkiMockServer::waitForClient(int timeout_ms) {
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

void EkiMockServer::dropClient() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (client_fd_ >= 0) {
    ::shutdown(client_fd_, SHUT_RDWR);
    ::close(client_fd_);
    client_fd_ = -1;
  }
  rx_buffer_.clear();
}

void EkiMockServer::pushHeartbeat() {
  std::lock_guard<std::mutex> lock(mutex_);
  sendStateLocked(0, true, kErrOk);
}

void EkiMockServer::setReady(bool ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  ready_ = ready;
}

void EkiMockServer::injectFault() {
  std::lock_guard<std::mutex> lock(mutex_);
  fault_ = true;
  rsi_active_ = false;
}

void EkiMockServer::setTool(double x, double y, double z, double a, double b,
                            double c) {
  std::lock_guard<std::mutex> lock(mutex_);
  tool_ = Frame6{x, y, z, a, b, c};
}

void EkiMockServer::setRespondToNext(bool respond) {
  std::lock_guard<std::mutex> lock(mutex_);
  respond_next_ = respond;
}

void EkiMockServer::sendMalformed() {
  std::lock_guard<std::mutex> lock(mutex_);
  static const char kBad[] = "<RobotState><Ack Seq=\"1\"</RobotState>";
  if (client_fd_ >= 0) {
#ifdef MSG_NOSIGNAL
    ::send(client_fd_, kBad, sizeof(kBad) - 1, MSG_NOSIGNAL);
#else
    ::send(client_fd_, kBad, sizeof(kBad) - 1, 0);
#endif
  }
}

bool EkiMockServer::rsiActive() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rsi_active_;
}

bool EkiMockServer::fault() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_;
}

int EkiMockServer::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

void EkiMockServer::sendStateLocked(std::uint32_t ack_seq, bool ok,
                                    int code) {
  // Caller holds mutex_.
  if (client_fd_ < 0) return;
  char buf[1024];
  const int n = std::snprintf(
      buf, sizeof(buf),
      "<RobotState>"
      "<Ack Seq=\"%u\" Ok=\"%d\" Code=\"%d\"/>"
      "<Prog Ready=\"%d\" RsiActive=\"%d\" Fault=\"%d\" Mode=\"%d\"/>"
      "<Tool X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\"/>"
      "</RobotState>",
      ack_seq, ok ? 1 : 0, code, ready_ ? 1 : 0, rsi_active_ ? 1 : 0,
      fault_ ? 1 : 0, mode_, tool_.x, tool_.y, tool_.z, tool_.a, tool_.b,
      tool_.c);
  if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(buf)) return;
#ifdef MSG_NOSIGNAL
  ::send(client_fd_, buf, static_cast<std::size_t>(n), MSG_NOSIGNAL);
#else
  ::send(client_fd_, buf, static_cast<std::size_t>(n), 0);
#endif
}

bool EkiMockServer::parseCommand(const char* data, std::size_t len,
                                 EkiCommand& out) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(data, len) != tinyxml2::XML_SUCCESS) return false;
  const tinyxml2::XMLElement* root = doc.FirstChildElement("RobotCommand");
  if (root == nullptr) return false;
  const tinyxml2::XMLElement* cmd = root->FirstChildElement("Cmd");
  const tinyxml2::XMLElement* tool = root->FirstChildElement("Tool");
  if (cmd == nullptr || tool == nullptr) return false;
  unsigned seq = 0;
  int action = 0;
  if (cmd->QueryUnsignedAttribute("Seq", &seq) != tinyxml2::XML_SUCCESS ||
      cmd->QueryIntAttribute("Action", &action) != tinyxml2::XML_SUCCESS ||
      cmd->QueryIntAttribute("Value", &out.value) != tinyxml2::XML_SUCCESS)
    return false;
  out.seq = seq;
  out.action = static_cast<EkiAction>(action);
  tool->QueryDoubleAttribute("X", &out.tool.x);
  tool->QueryDoubleAttribute("Y", &out.tool.y);
  tool->QueryDoubleAttribute("Z", &out.tool.z);
  tool->QueryDoubleAttribute("A", &out.tool.a);
  tool->QueryDoubleAttribute("B", &out.tool.b);
  tool->QueryDoubleAttribute("C", &out.tool.c);
  return true;
}

void EkiMockServer::handleCommand(const EkiCommand& cmd) {
  // Caller holds mutex_. Behavior table (Task 9 header).
  ++commands_;
  if (!respond_next_) {
    respond_next_ = true;
    return;
  }
  bool ok = true;
  int code = kErrOk;
  switch (cmd.action) {
    case EkiAction::START_RSI:
      if (fault_) {
        ok = false;
        code = kErrFaulted;
      } else if (!ready_) {
        ok = false;
        code = kErrNotReady;
      } else {
        rsi_active_ = true;
      }
      break;
    case EkiAction::STOP_RSI:
      rsi_active_ = false;
      break;
    case EkiAction::SET_MODE:
      if (fault_) {
        ok = false;
        code = kErrFaulted;
      } else {
        mode_ = cmd.value;
      }
      break;
    case EkiAction::RESET_FAULT:
      fault_ = false;
      break;
    case EkiAction::SET_TOOL_BASE:
      if (rsi_active_) {
        ok = false;
        code = kErrNotReady;  // no tool change while servoing
      } else {
        tool_ = cmd.tool;
      }
      break;
    case EkiAction::GET_TOOL:
    case EkiAction::QUERY_STATE:
      break;
  }
  sendStateLocked(cmd.seq, ok, code);
}

void EkiMockServer::run() {
  using clock = std::chrono::steady_clock;
  auto next_heartbeat = clock::now();
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
          rx_buffer_.clear();
        }
      }
      continue;
    }
    pollfd p{client, POLLIN, 0};
    if (::poll(&p, 1, kPollMs) > 0) {
      char buf[2048];
      const ssize_t n = ::recv(client, buf, sizeof(buf), 0);
      if (n <= 0) {
        dropClient();
        continue;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      rx_buffer_.append(buf, static_cast<std::size_t>(n));
      for (;;) {
        const std::size_t end = rx_buffer_.find(kCmdEndTag);
        if (end == std::string::npos) break;
        const std::size_t doc_len = end + sizeof(kCmdEndTag) - 1;
        EkiCommand cmd;
        if (parseCommand(rx_buffer_.data(), doc_len, cmd)) {
          handleCommand(cmd);
        }
        rx_buffer_.erase(0, doc_len);
      }
    }
    if (cfg_.heartbeat_period_s > 0.0 && clock::now() >= next_heartbeat) {
      pushHeartbeat();
      next_heartbeat =
          clock::now() + std::chrono::microseconds(static_cast<long>(
                             cfg_.heartbeat_period_s * 1e6));
    }
  }
}

}  // namespace kuka_eki
