#include "kuka_eki_bridge/eki_bridge_runtime.h"

#include <chrono>

namespace kuka_eki {

namespace {
void sleepSlices(double seconds, const std::atomic<bool>& keep_running) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::microseconds(static_cast<long>(seconds * 1e6));
  while (keep_running.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}  // namespace

EkiBridgeRuntime::EkiBridgeRuntime(const EkiBridgeConfig& cfg) : cfg_(cfg) {
  session_.configure(cfg_.session);
}

EkiBridgeRuntime::~EkiBridgeRuntime() { stop(); }

double EkiBridgeRuntime::nowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool EkiBridgeRuntime::start() {
  if (running_.load()) return false;
  running_ = true;
  thread_ = std::thread(&EkiBridgeRuntime::run, this);
  return true;
}

void EkiBridgeRuntime::stop() {
  running_ = false;
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
  transport_.close();
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
}

bool EkiBridgeRuntime::connectNow(int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex_);
  connect_requested_ = true;
  cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
               [this] { return connected_; });
  return connected_;
}

void EkiBridgeRuntime::resolveLocked(CommandOutcome outcome, int error_code) {
  // Caller holds mutex_.
  result_.outcome = outcome;
  result_.error_code = error_code;
  result_.state = session_.snapshot(nowS()).last_state;
  result_ready_ = true;
  cv_.notify_all();
}

ExecuteResult EkiBridgeRuntime::execute(EkiAction action, int value,
                                        const Frame6& tool,
                                        const Frame6& base) {
  std::lock_guard<std::mutex> serialize(command_mutex_);
  std::unique_lock<std::mutex> lock(mutex_);
  req_cmd_ = EkiCommand{};
  req_cmd_.seq = next_seq_++;
  if (next_seq_ == 0) next_seq_ = 1;  // 0 is the heartbeat marker
  req_cmd_.action = action;
  req_cmd_.value = value;
  req_cmd_.tool = tool;
  req_cmd_.base = base;
  req_active_ = true;
  result_ready_ = false;
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(
          static_cast<long>(cfg_.session.response_timeout_s * 1000.0) + 1000);
  cv_.wait_until(lock, deadline, [this] { return result_ready_; });
  req_active_ = false;
  if (!result_ready_) {
    // io thread never resolved (e.g. stop() during execute): terminal
    // timeout so callers never see PENDING semantics.
    ExecuteResult r;
    r.outcome = CommandOutcome::TIMEOUT;
    r.error_code = kErrNotConnected;
    return r;
  }
  return result_;
}

EkiBridgeStatus EkiBridgeRuntime::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  EkiBridgeStatus s;
  s.connected = connected_;
  s.reconnects = reconnects_;
  s.session = session_.snapshot(nowS());
  return s;
}

void EkiBridgeRuntime::run() {
  char buf[2048];
  while (running_.load()) {
    if (!transport_.connected()) {
      bool want_connect;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
        want_connect = cfg_.auto_reconnect || connect_requested_;
        if (!want_connect && req_active_ && !result_ready_) {
          resolveLocked(CommandOutcome::REJECTED, kErrNotConnected);
        }
      }
      if (!want_connect) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (transport_.connect(cfg_.kuka_ip, cfg_.eki_port,
                             cfg_.connect_timeout_ms)) {
        std::lock_guard<std::mutex> lock(mutex_);
        splitter_.reset();
        session_.reset();  // no command can be pending while disconnected
        connected_ = true;
        connect_requested_ = false;
        if (was_connected_once_) ++reconnects_;
        was_connected_once_ = true;
        cv_.notify_all();
      } else {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (req_active_ && !result_ready_) {
            resolveLocked(CommandOutcome::REJECTED, kErrNotConnected);
          }
        }
        sleepSlices(cfg_.reconnect_backoff_s, running_);
      }
      continue;
    }

    // Service a queued request once no command is in flight.
    char cmd_buf[1024];
    std::size_t cmd_len = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (req_active_ && !result_ready_ && !session_.commandPending()) {
        if (session_.beginCommand(req_cmd_.seq, nowS())) {
          cmd_len = serializeCommand(req_cmd_, cmd_buf, sizeof(cmd_buf));
        }
      }
    }
    if (cmd_len > 0 && !transport_.send(cmd_buf, cmd_len, 200)) {
      std::lock_guard<std::mutex> lock(mutex_);
      connected_ = false;
      session_.reset();
      if (req_active_ && !result_ready_) {
        resolveLocked(CommandOutcome::REJECTED, kErrNotConnected);
      }
      continue;
    }

    const int n =
        transport_.receive(buf, sizeof(buf), cfg_.receive_timeout_ms);
    std::lock_guard<std::mutex> lock(mutex_);
    if (n < 0) {
      connected_ = false;
      const CommandOutcome oc = session_.reset();
      if (oc != CommandOutcome::NONE && req_active_ && !result_ready_) {
        resolveLocked(CommandOutcome::TIMEOUT, kErrNotConnected);
      }
      continue;
    }
    if (n > 0) {
      splitter_.feed(buf, static_cast<std::size_t>(n),
                     [this](const char* d, std::size_t len) {
                       EkiStateFrame f;
                       if (!parseState(d, len, f)) {
                         session_.onBadFrame();
                         return;
                       }
                       const CommandOutcome oc = session_.onState(f, nowS());
                       if (oc != CommandOutcome::NONE && req_active_ &&
                           !result_ready_) {
                         resolveLocked(oc, oc == CommandOutcome::REJECTED
                                               ? f.ack_code
                                               : 0);
                       }
                     });
    }
    const CommandOutcome oc = session_.tick(nowS());
    if (oc == CommandOutcome::TIMEOUT && req_active_ && !result_ready_) {
      resolveLocked(oc, 0);
    }
  }
}

}  // namespace kuka_eki
