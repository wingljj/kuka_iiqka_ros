#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "sri_force_torque_driver/sri_frame.h"
#include "sri_force_torque_driver/sri_mock_server.h"
#include "sri_force_torque_driver/tcp_client_transport.h"

using sri::FtSample;
using sri::SriFrameAssembler;
using sri::SriMockConfig;
using sri::SriMockServer;
using sri::TcpClientTransport;

namespace {

// Bounded receive loop: keeps feeding the assembler until `want` samples
// arrived or the deadline passed. All waits bounded (<= 2 s total).
int receiveSamples(TcpClientTransport& t, SriFrameAssembler& a, FtSample* out,
                   int want, int deadline_ms = 1000) {
  int got = 0;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(deadline_ms);
  char buf[2048];
  while (got < want && std::chrono::steady_clock::now() < deadline) {
    const int n = t.receive(buf, sizeof(buf), 50);
    if (n < 0) break;
    if (n == 0) continue;
    got += a.feed(reinterpret_cast<const std::uint8_t*>(buf),
                  static_cast<std::size_t>(n), out + got, want - got);
  }
  return got;
}

void startStream(TcpClientTransport& t, SriMockServer& mock) {
  ASSERT_TRUE(t.send(sri::startStreamCommand(),
                     std::strlen(sri::startStreamCommand()), 200));
  ASSERT_TRUE(mock.waitForStartCommand(500));
}

}  // namespace

TEST(SriMock, ScriptedFramesReachClient) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  mock.setWrench(1.0f, 0.0f, 2.0f, 0.0f, -1.0f, 0.0f);
  mock.sendFrames(3);
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(receiveSamples(t, a, out, 3), 3);
  EXPECT_FLOAT_EQ(out[0].ch[0], 1.0f);
  EXPECT_FLOAT_EQ(out[2].ch[2], 2.0f);
  EXPECT_EQ(out[1].package_number, out[0].package_number + 1u);
}

TEST(SriMock, HoldsFramesUntilStartCommand) {
  SriMockServer mock{SriMockConfig{}};  // require_start_command = true
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.sendFrames(2);  // silently ignored: not streaming yet
  SriFrameAssembler a;
  FtSample out[4];
  EXPECT_EQ(receiveSamples(t, a, out, 1, 150), 0);
  startStream(t, mock);
  mock.sendFrames(2);
  EXPECT_EQ(receiveSamples(t, a, out, 2), 2);
}

TEST(SriMock, BadChecksumFrameIsCountedNotDecoded) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  mock.sendBadChecksumFrame();
  mock.sendFrames(1);
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(receiveSamples(t, a, out, 1), 1);
  EXPECT_EQ(a.stats().bad_checksum, 1u);
}

TEST(SriMock, GarbageBytesThenValidFrameRecovers) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  const char garbage[] = "\x01\x02\x03garbage\xAA";
  mock.sendRaw(garbage, sizeof(garbage) - 1);
  mock.sendFrames(1);
  SriFrameAssembler a;
  FtSample out[4];
  ASSERT_EQ(receiveSamples(t, a, out, 1), 1);
  EXPECT_GT(a.stats().skipped_bytes, 0u);
}

TEST(SriMock, DropClientDetectedByTransport) {
  SriMockServer mock{SriMockConfig{}};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  mock.dropClient();
  char buf[64];
  int rc = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
  while (std::chrono::steady_clock::now() < deadline) {
    rc = t.receive(buf, sizeof(buf), 50);
    if (rc != 0) break;
  }
  EXPECT_EQ(rc, -1);
  EXPECT_FALSE(t.connected());
}

TEST(SriMock, PacedModeStreamsWithoutScripting) {
  SriMockConfig cfg;
  cfg.rate_hz = 200.0;
  SriMockServer mock{cfg};
  ASSERT_TRUE(mock.start());
  TcpClientTransport t;
  ASSERT_TRUE(t.connect("127.0.0.1", mock.port(), 500));
  ASSERT_TRUE(mock.waitForClient(500));
  startStream(t, mock);
  SriFrameAssembler a;
  FtSample out[8];
  EXPECT_GE(receiveSamples(t, a, out, 5, 1000), 5);
}
