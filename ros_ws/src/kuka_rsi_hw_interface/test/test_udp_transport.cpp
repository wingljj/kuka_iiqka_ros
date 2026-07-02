#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "kuka_rsi_hw_interface/udp_transport.h"

using kuka_rsi::UdpTransport;

TEST(UdpTransport, BindsToKernelAssignedPort) {
  UdpTransport t;
  ASSERT_TRUE(t.bind("127.0.0.1", 0));
  EXPECT_GT(t.boundPort(), 0);
}

TEST(UdpTransport, ReceiveTimesOutWhenNoData) {
  UdpTransport t;
  ASSERT_TRUE(t.bind("127.0.0.1", 0));
  char buf[64];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 20), 0);  // 20 ms, no sender
}

TEST(UdpTransport, RoundTripBetweenTwoEndpoints) {
  UdpTransport server;
  UdpTransport client;
  ASSERT_TRUE(server.bind("127.0.0.1", 0));
  ASSERT_TRUE(client.bind("127.0.0.1", 0));

  const char msg[] = "<Rob Type=\"KUKA\"/>";
  ASSERT_TRUE(client.sendTo("127.0.0.1", server.boundPort(), msg,
                            std::strlen(msg)));
  char buf[128];
  const int n = server.receive(buf, sizeof(buf), 200);
  ASSERT_EQ(n, static_cast<int>(std::strlen(msg)));
  EXPECT_EQ(std::string(buf, n), msg);
}

TEST(UdpTransport, ReplyReachesLastSender) {
  UdpTransport server;
  UdpTransport client;
  ASSERT_TRUE(server.bind("127.0.0.1", 0));
  ASSERT_TRUE(client.bind("127.0.0.1", 0));

  const char ping[] = "ping";
  ASSERT_TRUE(client.sendTo("127.0.0.1", server.boundPort(), ping, 4));
  char buf[64];
  ASSERT_EQ(server.receive(buf, sizeof(buf), 200), 4);

  const char pong[] = "pong";
  ASSERT_TRUE(server.sendToLastSender(pong, 4));
  const int n = client.receive(buf, sizeof(buf), 200);
  ASSERT_EQ(n, 4);
  EXPECT_EQ(std::string(buf, n), "pong");
}

TEST(UdpTransport, SendToLastSenderFailsBeforeAnyReceive) {
  UdpTransport t;
  ASSERT_TRUE(t.bind("127.0.0.1", 0));
  EXPECT_FALSE(t.sendToLastSender("x", 1));
}

TEST(UdpTransport, OperationsFailWhenUnbound) {
  UdpTransport t;
  char buf[8];
  EXPECT_EQ(t.receive(buf, sizeof(buf), 10), -1);
  EXPECT_FALSE(t.sendTo("127.0.0.1", 1, "x", 1));
}
