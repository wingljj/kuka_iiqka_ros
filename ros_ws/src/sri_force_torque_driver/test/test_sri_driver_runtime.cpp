#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "sri_force_torque_driver/sri_driver_runtime.h"
#include "sri_force_torque_driver/sri_mock_server.h"

using sri::SriDriverConfig;
using sri::SriDriverRuntime;
using sri::SriMockConfig;
using sri::SriMockServer;
using sri::SriWrenchSample;

namespace {

// Collects callback samples across threads.
class Sink {
 public:
  void push(const SriWrenchSample& s) {
    std::lock_guard<std::mutex> lock(m_);
    samples_.push_back(s);
  }
  std::size_t count() const {
    std::lock_guard<std::mutex> lock(m_);
    return samples_.size();
  }
  SriWrenchSample at(std::size_t i) const {
    std::lock_guard<std::mutex> lock(m_);
    return samples_.at(i);
  }
  SriWrenchSample last() const {
    std::lock_guard<std::mutex> lock(m_);
    return samples_.back();
  }

 private:
  mutable std::mutex m_;
  std::vector<SriWrenchSample> samples_;
};

bool waitFor(const std::function<bool()>& pred, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

SriDriverConfig config(std::uint16_t port) {
  SriDriverConfig c;
  c.sensor_ip = "127.0.0.1";
  c.sensor_port = port;
  c.connect_timeout_ms = 200;
  c.receive_timeout_ms = 20;
  c.reconnect_backoff_s = 0.05;
  c.session.sample_timeout_s = 0.1;
  c.session.nominal_rate_hz = 200.0;
  c.session.zero_sample_count = 5;
  c.session.filter_cutoff_hz = 0.0;
  c.session.bias_limit_n = 0.0;
  return c;
}

SriMockConfig pacedMock() {
  SriMockConfig m;
  m.rate_hz = 200.0;
  return m;
}

}  // namespace

TEST(SriRuntime, StreamsAndDeliversSamples) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  mock.setWrench(1.5f, 0, 0, 0, 0, 0);
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 5; }, 2000));
  EXPECT_NEAR(sink.last().w.fx, 1.5, 1e-6);
  EXPECT_TRUE(rt.status().connected);
  rt.stop();
}

TEST(SriRuntime, StampsAreNonZeroAndMonotonic) {
  // Plan 3 follow-up 3 hardening: reception-instant stamps, never zero.
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 3; }, 2000));
  rt.stop();
  EXPECT_GT(sink.at(0).stamp_s, 0.0);
  EXPECT_GE(sink.at(1).stamp_s, sink.at(0).stamp_s);
  EXPECT_GE(sink.at(2).stamp_s, sink.at(1).stamp_s);
}

TEST(SriRuntime, RequestZeroBiasesSubsequentSamples) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  mock.setWrench(2.0f, 0, 0, 0, 0, 0);
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 2; }, 2000));
  ASSERT_TRUE(rt.requestZero(1000));  // averages 5 samples of constant 2.0
  const std::size_t base = sink.count();
  ASSERT_TRUE(waitFor([&] { return sink.count() >= base + 2; }, 2000));
  EXPECT_NEAR(sink.last().w.fx, 0.0, 1e-5);  // 2.0 - bias 2.0
  rt.stop();
}

TEST(SriRuntime, RequestZeroTimesOutWithoutStream) {
  SriMockConfig m;  // scripted mode + start-command gate: no frames flow
  SriMockServer mock{m};
  ASSERT_TRUE(mock.start());
  SriDriverRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 1000));
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(rt.requestZero(200));
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                .count(),
            1000);
  EXPECT_FALSE(rt.status().session.zero_active);  // capture was cancelled
  rt.stop();
}

TEST(SriRuntime, ReconnectsAfterDrop) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  Sink sink;
  SriDriverRuntime rt{config(mock.port())};
  rt.setSampleCallback([&sink](const SriWrenchSample& s) { sink.push(s); });
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return sink.count() >= 2; }, 2000));
  mock.dropClient();
  ASSERT_TRUE(waitFor([&] { return rt.status().reconnects >= 1; }, 2000));
  const std::size_t base = sink.count();
  ASSERT_TRUE(waitFor([&] { return sink.count() > base; }, 2000));
  rt.stop();
}

TEST(SriRuntime, SetFilterCutoffReflectedInStatus) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  SriDriverRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  EXPECT_TRUE(rt.setFilterCutoff(10.0));
  EXPECT_EQ(rt.status().session.filter_cutoff_hz, 10.0);
  EXPECT_FALSE(rt.setFilterCutoff(-1.0));  // negative rejected
  rt.stop();
}

TEST(SriRuntime, StopJoinsQuickly) {
  SriMockServer mock{pacedMock()};
  ASSERT_TRUE(mock.start());
  SriDriverRuntime rt{config(mock.port())};
  ASSERT_TRUE(rt.start());
  ASSERT_TRUE(waitFor([&] { return rt.status().connected; }, 1000));
  const auto t0 = std::chrono::steady_clock::now();
  rt.stop();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
  EXPECT_LT(ms, 1000);
}
