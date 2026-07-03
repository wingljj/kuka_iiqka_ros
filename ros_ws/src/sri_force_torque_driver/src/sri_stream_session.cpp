#include "sri_force_torque_driver/sri_stream_session.h"

namespace sri {

namespace {
sfc::Wrench toWrench(const FtSample& s) {
  sfc::Wrench w;
  w.fx = s.ch[0];
  w.fy = s.ch[1];
  w.fz = s.ch[2];
  w.tx = s.ch[3];
  w.ty = s.ch[4];
  w.tz = s.ch[5];
  return w;
}
}  // namespace

void SriStreamSession::configure(const SriSessionConfig& cfg) {
  cfg_ = cfg;
  filter_ = sfc::ForceTorqueFilter(cfg_.filter_cutoff_hz);
}

void SriStreamSession::reset() {
  assembler_.reset();
  filter_.reset();
  have_last_pn_ = false;
  zero_remaining_ = 0;    // an interrupted capture is abandoned, bias kept
  last_zero_ok_ = false;  // Plan 4 follow-up 4 (N1): never report the
                          // previous capture's success for this one
}

void SriStreamSession::startZero() {
  zero_remaining_ = cfg_.zero_sample_count;
  zero_accum_ = sfc::Wrench{};
}

void SriStreamSession::cancelZero() { zero_remaining_ = 0; }

void SriStreamSession::setFilterCutoff(double cutoff_hz) {
  cfg_.filter_cutoff_hz = cutoff_hz;
  filter_ = sfc::ForceTorqueFilter(cutoff_hz);
}

void SriStreamSession::finishZeroCapture() {
  const double n = static_cast<double>(cfg_.zero_sample_count);
  sfc::Wrench mean;
  mean.fx = zero_accum_.fx / n;
  mean.fy = zero_accum_.fy / n;
  mean.fz = zero_accum_.fz / n;
  mean.tx = zero_accum_.tx / n;
  mean.ty = zero_accum_.ty / n;
  mean.tz = zero_accum_.tz / n;
  if (cfg_.bias_limit_n > 0.0 && mean.forceNorm() > cfg_.bias_limit_n) {
    ++zero_rejects_;  // taring under load (legacy FTBia, decision 4)
    last_zero_ok_ = false;
    return;
  }
  bias_ = mean;
  filter_.reset();  // bias step must not smear through the filter state
  last_zero_ok_ = true;
}

int SriStreamSession::feed(const std::uint8_t* data, std::size_t len,
                           double now_s, SriWrenchSample* out, int max_out) {
  // 80 covers the worst-case burst of one 2048-byte read (66 x 31-byte
  // frames); anything beyond is counted by the assembler as dropped.
  FtSample raw[80];
  const int n = assembler_.feed(data, len, raw, 80);
  int written = 0;
  for (int i = 0; i < n; ++i) {
    const sfc::Wrench w = toWrench(raw[i]);
    if (have_last_pn_) {
      const std::uint16_t expected =
          static_cast<std::uint16_t>(last_pn_ + 1u);
      if (raw[i].package_number != expected) ++gaps_;
    }
    last_pn_ = raw[i].package_number;
    have_last_pn_ = true;
    ++samples_;
    last_sample_s_ = now_s;
    if (zero_remaining_ > 0) {
      zero_accum_.fx += w.fx;
      zero_accum_.fy += w.fy;
      zero_accum_.fz += w.fz;
      zero_accum_.tx += w.tx;
      zero_accum_.ty += w.ty;
      zero_accum_.tz += w.tz;
      if (--zero_remaining_ == 0) finishZeroCapture();
    }
    sfc::Wrench unbiased;
    unbiased.fx = w.fx - bias_.fx;
    unbiased.fy = w.fy - bias_.fy;
    unbiased.fz = w.fz - bias_.fz;
    unbiased.tx = w.tx - bias_.tx;
    unbiased.ty = w.ty - bias_.ty;
    unbiased.tz = w.tz - bias_.tz;
    const double dt = 1.0 / cfg_.nominal_rate_hz;
    const sfc::Wrench filtered = filter_.filter(unbiased, dt);
    if (written < max_out) {
      out[written].w = filtered;
      out[written].stamp_s = now_s;
      out[written].package_number = raw[i].package_number;
      ++written;
    }
  }
  return written;
}

SriStatusSnapshot SriStreamSession::status(double now_s) const {
  SriStatusSnapshot s;
  s.samples = samples_;
  s.bad_frames = assembler_.stats().bad_checksum + assembler_.stats().bad_length;
  s.package_gaps = gaps_;
  s.zero_active = zeroActive();
  s.zero_rejects = zero_rejects_;
  s.filter_cutoff_hz = cfg_.filter_cutoff_hz;
  s.bias = bias_;
  if (last_sample_s_ >= 0.0) {
    s.last_sample_age_s = now_s - last_sample_s_;
    s.streaming = s.last_sample_age_s <= cfg_.sample_timeout_s;
  }
  return s;
}

}  // namespace sri
