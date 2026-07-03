#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "sri_force_torque_driver/sri_stream_session.h"

using sri::SriSessionConfig;
using sri::SriStreamSession;
using sri::SriWrenchSample;

namespace {

constexpr double kNow = 50.0;  // arbitrary monotonic origin [s]

// Builds one valid frame for the given channels/package number. The
// checksum rule itself is pinned by the literal-byte tests of Task 2;
// this helper only reuses it.
std::vector<std::uint8_t> frame(const float ch[6], std::uint16_t pn) {
  std::vector<std::uint8_t> f = {0xAA, 0x55, 0x00, 0x1B,
                                 static_cast<std::uint8_t>(pn >> 8),
                                 static_cast<std::uint8_t>(pn & 0xFF)};
  unsigned sum = 0;
  for (int i = 0; i < 6; ++i) {
    std::uint8_t b[4];
    std::memcpy(b, &ch[i], 4);
    for (int k = 0; k < 4; ++k) {
      f.push_back(b[k]);
      sum += b[k];
    }
  }
  f.push_back(static_cast<std::uint8_t>(sum & 0xFF));
  return f;
}

std::vector<std::uint8_t> frameFx(float fx, std::uint16_t pn) {
  const float ch[6] = {fx, 0, 0, 0, 0, 0};
  return frame(ch, pn);
}

SriSessionConfig config() {
  SriSessionConfig c;
  c.sample_timeout_s = 0.1;
  c.nominal_rate_hz = 250.0;
  c.zero_sample_count = 2;
  c.filter_cutoff_hz = 0.0;
  c.bias_limit_n = 0.0;
  return c;
}

int feedOne(SriStreamSession& s, const std::vector<std::uint8_t>& bytes,
            double now, SriWrenchSample* out) {
  return s.feed(bytes.data(), bytes.size(), now, out, 8);
}

}  // namespace

TEST(SriSession, RawPassthroughByDefault) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  ASSERT_EQ(feedOne(s, frameFx(1.5f, 1), kNow, out), 1);
  EXPECT_NEAR(out[0].w.fx, 1.5, 1e-6);
  EXPECT_EQ(out[0].package_number, 1u);
}

TEST(SriSession, StampIsReceptionTime) {
  // Plan 3 follow-up 3: the stamp must be the reception instant handed
  // to feed(), never zero.
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  ASSERT_EQ(feedOne(s, frameFx(1.0f, 1), 123.456, out), 1);
  EXPECT_EQ(out[0].stamp_s, 123.456);
}

TEST(SriSession, ZeroCaptureAveragesAndSubtracts) {
  SriStreamSession s;
  s.configure(config());  // zero_sample_count = 2
  SriWrenchSample out[8];
  s.startZero();
  EXPECT_TRUE(s.zeroActive());
  feedOne(s, frameFx(1.0f, 1), kNow, out);
  feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out);
  EXPECT_FALSE(s.zeroActive());
  EXPECT_TRUE(s.lastZeroAccepted());
  // bias = (1.0 + 2.0) / 2 = 1.5; next raw 2.0 -> 0.5
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 3), kNow + 0.008, out), 1);
  EXPECT_NEAR(out[0].w.fx, 0.5, 1e-6);
  EXPECT_NEAR(s.status(kNow + 0.008).bias.fx, 1.5, 1e-6);
}

TEST(SriSession, ZeroRejectedAboveBiasLimit) {
  SriSessionConfig c = config();
  c.bias_limit_n = 1.0;  // legacy FTBia semantics (decision 4)
  SriStreamSession s;
  s.configure(c);
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(2.0f, 1), kNow, out);
  feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out);
  EXPECT_FALSE(s.zeroActive());
  EXPECT_FALSE(s.lastZeroAccepted());
  EXPECT_EQ(s.status(kNow + 0.004).zero_rejects, 1u);
  // bias unchanged (0): raw 2.0 passes through unchanged.
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 3), kNow + 0.008, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-6);
}

TEST(SriSession, CancelZeroKeepsOldBias) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(1.0f, 1), kNow, out);  // capture 1/2
  s.cancelZero();
  EXPECT_FALSE(s.zeroActive());
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-6);  // no bias was applied
}

TEST(SriSession, FilterHalvesStepWithConstructedAlpha) {
  SriSessionConfig c = config();
  // dt = 1/250 = 0.004; cutoff = 1/(2*pi*0.004) makes alpha exactly 0.5.
  c.filter_cutoff_hz = 1.0 / (2.0 * M_PI * 0.004);
  SriStreamSession s;
  s.configure(c);
  SriWrenchSample out[8];
  ASSERT_EQ(feedOne(s, frameFx(0.0f, 1), kNow, out), 1);
  EXPECT_NEAR(out[0].w.fx, 0.0, 1e-9);  // first sample initializes
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out), 1);
  EXPECT_NEAR(out[0].w.fx, 1.0, 1e-9);  // 0 + 0.5 * (2 - 0)
}

TEST(SriSession, SetFilterCutoffRebuildsAndResets) {
  SriSessionConfig c = config();
  c.filter_cutoff_hz = 1.0 / (2.0 * M_PI * 0.004);
  SriStreamSession s;
  s.configure(c);
  SriWrenchSample out[8];
  feedOne(s, frameFx(0.0f, 1), kNow, out);
  s.setFilterCutoff(0.0);  // back to passthrough; state discarded
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 2), kNow + 0.004, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-9);
  EXPECT_EQ(s.status(kNow + 0.004).filter_cutoff_hz, 0.0);
}

TEST(SriSession, PackageGapCountedWrapIsNot) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  feedOne(s, frameFx(0, 1), kNow, out);
  feedOne(s, frameFx(0, 3), kNow, out);  // 1 -> 3: one gap
  EXPECT_EQ(s.status(kNow).package_gaps, 1u);
  SriStreamSession s2;
  s2.configure(config());
  feedOne(s2, frameFx(0, 65535), kNow, out);
  feedOne(s2, frameFx(0, 0), kNow, out);  // wraparound: no gap
  EXPECT_EQ(s2.status(kNow).package_gaps, 0u);
}

TEST(SriSession, StreamingFlagFollowsSampleAge) {
  SriStreamSession s;
  s.configure(config());  // sample_timeout_s = 0.1
  SriWrenchSample out[8];
  EXPECT_FALSE(s.status(kNow).streaming);
  EXPECT_EQ(s.status(kNow).last_sample_age_s, -1.0);
  feedOne(s, frameFx(0, 1), kNow, out);
  EXPECT_TRUE(s.status(kNow + 0.05).streaming);
  EXPECT_NEAR(s.status(kNow + 0.05).last_sample_age_s, 0.05, 1e-9);
  EXPECT_FALSE(s.status(kNow + 0.2).streaming);
}

TEST(SriSession, BadFrameSurfacesInStatus) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  std::vector<std::uint8_t> bad = frameFx(1.0f, 1);
  bad.back() ^= 0xFF;  // corrupt checksum
  EXPECT_EQ(s.feed(bad.data(), bad.size(), kNow, out, 8), 0);
  EXPECT_EQ(s.status(kNow).bad_frames, 1u);
  EXPECT_EQ(s.status(kNow).samples, 0u);
}

TEST(SriSession, ResetKeepsBiasAndResyncsAssembler) {
  SriStreamSession s;
  s.configure(config());
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(1.0f, 1), kNow, out);
  feedOne(s, frameFx(1.0f, 2), kNow, out);  // bias = 1.0
  std::vector<std::uint8_t> half = frameFx(2.0f, 3);
  half.resize(10);  // truncated frame left in the assembler
  s.feed(half.data(), half.size(), kNow, out, 8);
  s.reset();  // reconnect: resync, keep bias
  ASSERT_EQ(feedOne(s, frameFx(2.0f, 4), kNow + 0.1, out), 1);
  EXPECT_NEAR(out[0].w.fx, 1.0, 1e-6);  // 2.0 - kept bias 1.0
}

TEST(SriSession, ResetClearsLastZeroAccepted) {
  // Plan 4 follow-up 4 (N1): a capture killed by a reconnect must not let
  // requestZero() report the PREVIOUS capture's success.
  SriStreamSession s;
  s.configure(config());  // zero_sample_count = 2
  SriWrenchSample out[8];
  s.startZero();
  feedOne(s, frameFx(2.0f, 1), kNow, out);
  feedOne(s, frameFx(4.0f, 2), kNow, out);
  ASSERT_TRUE(s.lastZeroAccepted());  // capture #1: bias.fx = 3.0

  s.startZero();                      // capture #2 begins...
  feedOne(s, frameFx(2.0f, 3), kNow, out);
  s.reset();                          // ...and dies with the connection
  EXPECT_FALSE(s.zeroActive());
  EXPECT_FALSE(s.lastZeroAccepted());  // must NOT echo capture #1

  // The old bias survives reset (physical sensor state unchanged).
  ASSERT_EQ(feedOne(s, frameFx(5.0f, 1), kNow, out), 1);
  EXPECT_NEAR(out[0].w.fx, 2.0, 1e-6);  // 5.0 - 3.0
}
