#include "sri_force_torque_driver/sri_driver_runtime.h"

#include <chrono>
#include <cstring>

namespace sri {

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

SriDriverRuntime::SriDriverRuntime(const SriDriverConfig& cfg) : cfg_(cfg) {
  session_.configure(cfg_.session);
}

SriDriverRuntime::~SriDriverRuntime() { stop(); }

double SriDriverRuntime::nowS() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void SriDriverRuntime::setSampleCallback(SampleCallback cb) {
  callback_ = std::move(cb);
}

bool SriDriverRuntime::start() {
  if (running_.load()) return false;
  running_ = true;
  thread_ = std::thread(&SriDriverRuntime::run, this);
  return true;
}

void SriDriverRuntime::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  transport_.close();
  std::lock_guard<std::mutex> lock(mutex_);
  connected_ = false;
}

bool SriDriverRuntime::requestZero(int timeout_ms) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    session_.startZero();
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!session_.zeroActive()) return session_.lastZeroAccepted();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!session_.zeroActive()) return session_.lastZeroAccepted();
  session_.cancelZero();
  return false;
}

bool SriDriverRuntime::setFilterCutoff(double cutoff_hz) {
  if (cutoff_hz < 0.0) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  session_.setFilterCutoff(cutoff_hz);
  return true;
}

SriDriverStatus SriDriverRuntime::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  SriDriverStatus s;
  s.connected = connected_;
  s.reconnects = reconnects_;
  s.session = session_.status(nowS());
  return s;
}

void SriDriverRuntime::run() {
  char buf[2048];
  SriWrenchSample samples[80];
  while (running_.load()) {
    if (!transport_.connected()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
      }
      if (!transport_.connect(cfg_.sensor_ip, cfg_.sensor_port,
                              cfg_.connect_timeout_ms)) {
        sleepSlices(cfg_.reconnect_backoff_s, running_);
        continue;
      }
      transport_.send(startStreamCommand(),
                      std::strlen(startStreamCommand()), 200);
      std::lock_guard<std::mutex> lock(mutex_);
      session_.reset();
      connected_ = true;
      if (was_connected_once_) ++reconnects_;
      was_connected_once_ = true;
    }
    const int n = transport_.receive(buf, sizeof(buf),
                                     cfg_.receive_timeout_ms);
    if (n < 0) continue;  // next iteration handles reconnect
    if (n == 0) continue;
    int count;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      count = session_.feed(reinterpret_cast<const std::uint8_t*>(buf),
                            static_cast<std::size_t>(n), nowS(), samples, 80);
    }
    for (int i = 0; i < count; ++i) {
      if (callback_) callback_(samples[i]);
    }
  }
}

}  // namespace sri
