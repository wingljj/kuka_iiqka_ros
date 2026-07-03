#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "sri_force_torque_driver/sri_frame.h"

using sri::AssemblerStats;
using sri::FtSample;
using sri::SriFrameAssembler;

namespace {

// Canonical frame, checksum hand-derived in the plan (Task 2 header):
// Fx=1.0 Fy=0 Fz=2.0 Mx=0 My=-1.0 Mz=0, PN=1.
// sum(data) = (0x80+0x3F) + 0x40 + (0x80+0xBF) = 0x23E -> 0x3E.
const std::uint8_t kFrameA[31] = {
    0xAA, 0x55, 0x00, 0x1B, 0x00, 0x01,
    0x00, 0x00, 0x80, 0x3F,   // Fx = 1.0f (LE)
    0x00, 0x00, 0x00, 0x00,   // Fy = 0.0f
    0x00, 0x00, 0x00, 0x40,   // Fz = 2.0f
    0x00, 0x00, 0x00, 0x00,   // Mx = 0.0f
    0x00, 0x00, 0x80, 0xBF,   // My = -1.0f
    0x00, 0x00, 0x00, 0x00,   // Mz = 0.0f
    0x3E};

// Second frame: Fx=0.5 (00 00 00 3F), rest 0, PN=2 -> checksum 0x3F.
const std::uint8_t kFrameB[31] = {
    0xAA, 0x55, 0x00, 0x1B, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x3F,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0x3F};

int feedAll(SriFrameAssembler& a, const std::uint8_t* data, std::size_t len,
            FtSample* out, int max_out) {
  return a.feed(data, len, out, max_out);
}

}  // namespace

TEST(SriFrame, SingleFrameDecodes) {
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(feedAll(a, kFrameA, sizeof(kFrameA), out, 4), 1);
  EXPECT_FLOAT_EQ(out[0].ch[0], 1.0f);
  EXPECT_FLOAT_EQ(out[0].ch[1], 0.0f);
  EXPECT_FLOAT_EQ(out[0].ch[2], 2.0f);
  EXPECT_FLOAT_EQ(out[0].ch[4], -1.0f);
  EXPECT_EQ(out[0].package_number, 1u);
  EXPECT_EQ(a.stats().frames, 1u);
}

TEST(SriFrame, PackageNumberIsBigEndian) {
  std::uint8_t f[31];
  std::memcpy(f, kFrameA, sizeof(f));
  f[4] = 0x01;  // PN = 0x0102 = 258
  f[5] = 0x02;
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(f, sizeof(f), out, 1), 1);
  EXPECT_EQ(out[0].package_number, 258u);
}

TEST(SriFrame, BadChecksumCountedNoSample) {
  std::uint8_t f[31];
  std::memcpy(f, kFrameA, sizeof(f));
  f[30] = 0x00;  // corrupt checksum (correct value is 0x3E)
  SriFrameAssembler a;
  FtSample out[1];
  EXPECT_EQ(a.feed(f, sizeof(f), out, 1), 0);
  EXPECT_EQ(a.stats().bad_checksum, 1u);
  EXPECT_EQ(a.stats().frames, 0u);
}

TEST(SriFrame, RecoversAfterBadFrame) {
  std::uint8_t bad[31];
  std::memcpy(bad, kFrameA, sizeof(bad));
  bad[30] = 0x00;
  SriFrameAssembler a;
  FtSample out[2];
  EXPECT_EQ(a.feed(bad, sizeof(bad), out, 2), 0);
  EXPECT_EQ(a.feed(kFrameB, sizeof(kFrameB), out, 2), 1);
  EXPECT_FLOAT_EQ(out[0].ch[0], 0.5f);
  EXPECT_EQ(out[0].package_number, 2u);
}

TEST(SriFrame, SplitAcrossFeedsByteByByte) {
  SriFrameAssembler a;
  FtSample out[1];
  int total = 0;
  for (std::size_t i = 0; i < sizeof(kFrameA); ++i)
    total += a.feed(&kFrameA[i], 1, out, 1);
  EXPECT_EQ(total, 1);
  EXPECT_FLOAT_EQ(out[0].ch[2], 2.0f);
}

TEST(SriFrame, MultipleFramesInOneFeed) {
  std::vector<std::uint8_t> buf(kFrameA, kFrameA + sizeof(kFrameA));
  buf.insert(buf.end(), kFrameB, kFrameB + sizeof(kFrameB));
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 4), 2);
  EXPECT_EQ(out[0].package_number, 1u);
  EXPECT_EQ(out[1].package_number, 2u);
}

TEST(SriFrame, AsciiAckBeforeSyncIsSkipped) {
  const char* ack = "ACK+GSD=OK\r\n";
  std::vector<std::uint8_t> buf(ack, ack + std::strlen(ack));
  buf.insert(buf.end(), kFrameA, kFrameA + sizeof(kFrameA));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
  EXPECT_EQ(a.stats().skipped_bytes, std::strlen(ack));
}

TEST(SriFrame, WrongDeclaredLengthCountedAsBadLength) {
  std::uint8_t f[31];
  std::memcpy(f, kFrameA, sizeof(f));
  f[3] = 0x10;  // declared 0x0010 != 27
  SriFrameAssembler a;
  FtSample out[1];
  EXPECT_EQ(a.feed(f, sizeof(f), out, 1), 0);
  EXPECT_EQ(a.stats().bad_length, 1u);
}

TEST(SriFrame, AaNotFollowedBy55Resyncs) {
  std::vector<std::uint8_t> buf = {0xAA, 0x41};
  buf.insert(buf.end(), kFrameA, kFrameA + sizeof(kFrameA));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
}

TEST(SriFrame, RepeatedAaStillSyncs) {
  // AA AA 55 ...: the second AA must be treated as a candidate sync-1.
  std::vector<std::uint8_t> buf = {0xAA};
  buf.insert(buf.end(), kFrameA, kFrameA + sizeof(kFrameA));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
}

TEST(SriFrame, FramesBeyondMaxOutAreDroppedAndCounted) {
  std::vector<std::uint8_t> buf(kFrameA, kFrameA + sizeof(kFrameA));
  buf.insert(buf.end(), kFrameB, kFrameB + sizeof(kFrameB));
  SriFrameAssembler a;
  FtSample out[1];
  ASSERT_EQ(a.feed(buf.data(), buf.size(), out, 1), 1);
  EXPECT_EQ(a.stats().dropped, 1u);
  EXPECT_EQ(a.stats().frames, 2u);  // both frames were valid
}

TEST(SriFrame, CommandStringsAreCrlfTerminated) {
  EXPECT_STREQ(sri::startStreamCommand(), "AT+GSD\r\n");
  EXPECT_STREQ(sri::stopStreamCommand(), "AT+GSD=STOP\r\n");
}
