#pragma once

#include <cstddef>
#include <cstdint>

#include "soft_force_control_core/force_torque_filter.h"
#include "soft_force_control_core/types.h"
#include "sri_force_torque_driver/sri_frame.h"

namespace sri {

struct SriSessionConfig {
  double sample_timeout_s{0.1};   // streaming -> stalled threshold
  double nominal_rate_hz{250.0};  // fixed filter dt = 1/rate (decision 3)
  int zero_sample_count{100};     // samples averaged by one tare capture
  double filter_cutoff_hz{0.0};   // 0 = raw passthrough (default)
  double bias_limit_n{0.0};       // reject tare above this force norm; 0 = off
};

// One published-ready sample: bias-subtracted, optionally filtered.
struct SriWrenchSample {
  sfc::Wrench w;                    // N / Nm
  double stamp_s{0};                // reception time handed to feed()
  std::uint16_t package_number{0};
};

struct SriStatusSnapshot {
  bool streaming{false};
  double last_sample_age_s{-1.0};  // -1 until the first valid sample
  std::uint64_t samples{0};
  std::uint64_t bad_frames{0};     // checksum + length failures
  std::uint64_t package_gaps{0};
  bool zero_active{false};
  std::uint32_t zero_rejects{0};
  double filter_cutoff_hz{0.0};
  sfc::Wrench bias;
};

// Pure per-connection stream logic (spec 5.6 + decisions 2-4): raw TCP
// bytes in, timestamped wrench samples out. Per sample: raw value ->
// zero-capture accumulation (raw!) -> bias subtraction -> optional
// first-order low-pass (fixed dt = 1/nominal_rate_hz) -> output.
// reset() is the reconnect hook: resync assembler, reset filter state,
// KEEP the bias (the physical sensor state did not change). No ROS, no
// allocation, single-threaded (the runtime serializes access).
class SriStreamSession {
 public:
  void configure(const SriSessionConfig& cfg);
  void reset();

  int feed(const std::uint8_t* data, std::size_t len, double now_s,
           SriWrenchSample* out, int max_out);

  void startZero();
  void cancelZero();
  bool zeroActive() const { return zero_remaining_ > 0; }
  bool lastZeroAccepted() const { return last_zero_ok_; }

  // Rebuild + implicit reset (Plan 1 follow-up 3 semantics).
  void setFilterCutoff(double cutoff_hz);

  SriStatusSnapshot status(double now_s) const;

 private:
  void finishZeroCapture();

  SriSessionConfig cfg_;
  SriFrameAssembler assembler_;
  sfc::ForceTorqueFilter filter_{0.0};
  sfc::Wrench bias_;
  sfc::Wrench zero_accum_;
  int zero_remaining_{0};
  bool last_zero_ok_{false};
  std::uint32_t zero_rejects_{0};
  bool have_last_pn_{false};
  std::uint16_t last_pn_{0};
  double last_sample_s_{-1.0};
  std::uint64_t samples_{0};
  std::uint64_t gaps_{0};
};

}  // namespace sri
