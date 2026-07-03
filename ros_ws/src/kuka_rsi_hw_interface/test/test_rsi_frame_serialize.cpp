#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <cstring>
#include <string>

#include "kuka_rsi_hw_interface/rsi_frame.h"

using kuka_rsi::SenFrame;
using kuka_rsi::serializeSenFrame;

namespace {
SenFrame sample() {
  SenFrame f;
  f.x = 0.1234;
  f.y = -0.5;
  f.z = 0.0;
  f.a = 0.01;
  f.b = -0.02;
  f.c = 0.03;
  f.stop = 0;
  f.watchdog = 42;
  f.ipoc = 4894312;
  return f;
}
}  // namespace

TEST(SenFrameSerialize, ProducesParsableXmlWithAllFields) {
  char buf[1024];
  const std::size_t n = serializeSenFrame(sample(), buf, sizeof(buf));
  ASSERT_GT(n, 0u);
  EXPECT_EQ(n, std::strlen(buf));  // NUL-terminated, length consistent

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(doc.Parse(buf, n), tinyxml2::XML_SUCCESS);
  const tinyxml2::XMLElement* sen = doc.FirstChildElement("Sen");
  ASSERT_NE(sen, nullptr);
  EXPECT_STREQ(sen->Attribute("Type"), "ROS");

  const tinyxml2::XMLElement* rkorr = sen->FirstChildElement("RKorr");
  ASSERT_NE(rkorr, nullptr);
  EXPECT_NEAR(rkorr->DoubleAttribute("X"), 0.1234, 1e-9);
  EXPECT_NEAR(rkorr->DoubleAttribute("Y"), -0.5, 1e-9);
  EXPECT_NEAR(rkorr->DoubleAttribute("C"), 0.03, 1e-9);

  const tinyxml2::XMLElement* stop = sen->FirstChildElement("Stop");
  ASSERT_NE(stop, nullptr);
  EXPECT_EQ(stop->IntAttribute("S"), 0);

  const tinyxml2::XMLElement* wd = sen->FirstChildElement("Watchdog");
  ASSERT_NE(wd, nullptr);
  // Int64Attribute: tinyxml2 6.2 has no Unsigned64Attribute (added in 7.0).
  EXPECT_EQ(wd->Int64Attribute("W"), 42);

  const tinyxml2::XMLElement* ipoc = sen->FirstChildElement("IPOC");
  ASSERT_NE(ipoc, nullptr);
  EXPECT_STREQ(ipoc->GetText(), "4894312");
}

TEST(SenFrameSerialize, EchoesIpocVerbatim) {
  SenFrame f = sample();
  f.ipoc = 18446744073709551615ull;  // uint64 max survives round trip
  char buf[1024];
  ASSERT_GT(serializeSenFrame(f, buf, sizeof(buf)), 0u);
  EXPECT_NE(std::strstr(buf, "<IPOC>18446744073709551615</IPOC>"), nullptr);
}

TEST(SenFrameSerialize, StopFlagSerialized) {
  SenFrame f = sample();
  f.stop = 1;
  char buf[1024];
  ASSERT_GT(serializeSenFrame(f, buf, sizeof(buf)), 0u);
  EXPECT_NE(std::strstr(buf, "<Stop S=\"1\"/>"), nullptr);
}

TEST(SenFrameSerialize, FourDecimalFixedFormat) {
  // Legacy ExternalData.xml uses fixed 4-decimal values; keep that format.
  SenFrame f;  // all zeros
  char buf[1024];
  ASSERT_GT(serializeSenFrame(f, buf, sizeof(buf)), 0u);
  EXPECT_NE(std::strstr(buf, "X=\"0.0000\""), nullptr);
  EXPECT_NE(std::strstr(buf, "C=\"0.0000\""), nullptr);
}

TEST(SenFrameSerialize, ReturnsZeroWhenBufferTooSmall) {
  char tiny[16];
  EXPECT_EQ(serializeSenFrame(sample(), tiny, sizeof(tiny)), 0u);
  char none[1];
  EXPECT_EQ(serializeSenFrame(sample(), none, sizeof(none)), 0u);
}
