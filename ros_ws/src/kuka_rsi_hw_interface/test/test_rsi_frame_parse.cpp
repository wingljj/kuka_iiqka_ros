#include <gtest/gtest.h>

#include <cstring>

#include "kuka_rsi_hw_interface/rsi_frame.h"

using kuka_rsi::RobFrame;
using kuka_rsi::parseRobFrame;

namespace {
// Frame layout per spec section 6.1 (KUKA -> ROS).
const char kGoodFrame[] =
    "<Rob Type=\"KUKA\">"
    "<RIst X=\"12.5\" Y=\"-3.25\" Z=\"800.0\" A=\"90.0\" B=\"-45.5\" "
    "C=\"179.9\"/>"
    "<AIPos A1=\"1.1\" A2=\"-90.2\" A3=\"88.3\" A4=\"0.4\" A5=\"45.5\" "
    "A6=\"-0.6\"/>"
    "<Delay D=\"2\"/>"
    "<Mode M=\"1\"/>"
    "<IPOC>4894312</IPOC>"
    "</Rob>";

bool parse(const char* s, RobFrame& out) {
  return parseRobFrame(s, std::strlen(s), out);
}
}  // namespace

TEST(RobFrameParse, ParsesCartesianPose) {
  RobFrame f;
  ASSERT_TRUE(parse(kGoodFrame, f));
  EXPECT_DOUBLE_EQ(f.x, 12.5);
  EXPECT_DOUBLE_EQ(f.y, -3.25);
  EXPECT_DOUBLE_EQ(f.z, 800.0);
  EXPECT_DOUBLE_EQ(f.a, 90.0);
  EXPECT_DOUBLE_EQ(f.b, -45.5);
  EXPECT_DOUBLE_EQ(f.c, 179.9);
}

TEST(RobFrameParse, ParsesJointAnglesDelayModeIpoc) {
  RobFrame f;
  ASSERT_TRUE(parse(kGoodFrame, f));
  EXPECT_DOUBLE_EQ(f.axis_deg[0], 1.1);
  EXPECT_DOUBLE_EQ(f.axis_deg[1], -90.2);
  EXPECT_DOUBLE_EQ(f.axis_deg[5], -0.6);
  EXPECT_DOUBLE_EQ(f.delay, 2.0);
  EXPECT_EQ(f.mode, 1);
  EXPECT_EQ(f.ipoc, 4894312u);
}

TEST(RobFrameParse, DelayAndModeAreOptional) {
  const char frame[] =
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>";
  RobFrame f;
  ASSERT_TRUE(parse(frame, f));
  EXPECT_DOUBLE_EQ(f.delay, 0.0);
  EXPECT_EQ(f.mode, 0);
  EXPECT_EQ(f.ipoc, 1u);
}

TEST(RobFrameParse, RejectsMalformedXml) {
  RobFrame f;
  EXPECT_FALSE(parse("<Rob Type=\"KUKA\"><RIst X=\"1\"", f));
  EXPECT_FALSE(parse("not xml at all", f));
  EXPECT_FALSE(parseRobFrame(nullptr, 0, f));
}

TEST(RobFrameParse, RejectsMissingMandatoryElements) {
  RobFrame f;
  // Missing IPOC.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "</Rob>",
      f));
  // Missing RIst.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
  // Missing AIPos.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
  // Wrong root element.
  EXPECT_FALSE(parse("<Sen Type=\"ROS\"><IPOC>1</IPOC></Sen>", f));
}

TEST(RobFrameParse, RejectsIncompleteAttributeSets) {
  RobFrame f;
  // RIst missing C attribute.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
  // AIPos missing A6 attribute.
  EXPECT_FALSE(parse(
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "<AIPos A1=\"0\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\"/>"
      "<IPOC>1</IPOC>"
      "</Rob>",
      f));
}

TEST(RobFrameParse, ValidFlagTracksResult) {
  RobFrame f;
  EXPECT_FALSE(f.valid);  // default-constructed
  ASSERT_TRUE(parse(kGoodFrame, f));
  EXPECT_TRUE(f.valid);
  EXPECT_FALSE(parse("garbage", f));
  EXPECT_FALSE(f.valid);  // reset on failed parse
}
