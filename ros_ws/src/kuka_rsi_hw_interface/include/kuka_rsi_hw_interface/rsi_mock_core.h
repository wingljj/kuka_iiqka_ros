#pragma once

#include <cstddef>
#include <cstdint>

namespace kuka_rsi {

// Initial pose and IPOC settings for the mock KRC.
struct MockConfig {
  double x0{0}, y0{0}, z0{800.0}, a0{0}, b0{0}, c0{0};
  std::uint64_t ipoc_start{1000};
  std::uint64_t ipoc_step{4};
};

struct MockStats {
  std::uint64_t frames_sent{0};
  std::uint64_t replies_received{0};
  std::uint64_t reply_timeouts{0};
  std::uint64_t ipoc_echo_errors{0};
  std::uint64_t parse_errors{0};
  int last_stop{0};
  std::uint64_t last_watchdog{0};
};

// Deterministic KRC-side RSI behavior model (spec section 15.2), no
// networking: emits <Rob> state frames and integrates <Sen> RKorr replies
// into its pose when the IPOC echo matches the frame just sent. Axis
// angles are reported as zero (no kinematics in this plan).
class RsiMockCore {
 public:
  explicit RsiMockCore(const MockConfig& cfg);

  // Serializes the current state frame and advances IPOC for the next
  // cycle. Returns payload length, 0 if the buffer is too small.
  std::size_t buildStateFrame(char* buf, std::size_t size);

  // Parses a <Sen> reply. On correct IPOC echo, integrates RKorr into the
  // pose and records stop/watchdog; returns true. On parse failure or
  // wrong echo, bumps the matching counter and returns false.
  bool applyReply(const char* data, std::size_t len);

  void noteReplyTimeout() { ++stats_.reply_timeouts; }

  void setPose(double x, double y, double z, double a, double b, double c) {
    x_ = x; y_ = y; z_ = z; a_ = a; b_ = b; c_ = c;
  }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double a() const { return a_; }
  double b() const { return b_; }
  double c() const { return c_; }
  const MockStats& stats() const { return stats_; }

 private:
  double x_, y_, z_, a_, b_, c_;
  std::uint64_t next_ipoc_;
  std::uint64_t ipoc_step_;
  std::uint64_t awaited_ipoc_{0};
  bool awaiting_{false};
  MockStats stats_;
};

}  // namespace kuka_rsi
