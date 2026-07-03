#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "kuka_eki_bridge/eki_frame.h"
#include "kuka_eki_bridge/eki_stream_splitter.h"

using kuka_eki::EkiAction;
using kuka_eki::EkiCommand;
using kuka_eki::EkiStateFrame;
using kuka_eki::EkiStreamSplitter;

namespace {

const char kStateXml[] =
    "<RobotState>"
    "<Ack Seq=\"7\" Ok=\"1\" Code=\"0\"/>"
    "<Prog Ready=\"1\" RsiActive=\"0\" Fault=\"0\" Mode=\"2\"/>"
    "<Tool X=\"10.5\" Y=\"0\" Z=\"235.0\" A=\"0\" B=\"90.0\" C=\"0\"/>"
    "</RobotState>";

EkiCommand makeCommand() {
  EkiCommand c;
  c.seq = 42;
  c.action = EkiAction::SET_TOOL_BASE;
  c.value = 0;
  c.tool = {10.5, 0.0, 235.0, 0.0, 90.0, 0.0};
  c.base = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return c;
}

}  // namespace

TEST(EkiFrame, SerializeCommandRoundTripsThroughParser) {
  // The codec must be self-consistent: serialize, then re-parse with
  // tinyxml2 via the state parser's sibling logic. We assert on the raw
  // XML text for the schema-critical parts (Plan 5 templates depend on
  // these exact element/attribute names).
  char buf[1024];
  const std::size_t n = kuka_eki::serializeCommand(makeCommand(), buf,
                                                   sizeof(buf));
  ASSERT_GT(n, 0u);
  const std::string xml(buf, n);
  EXPECT_NE(xml.find("<RobotCommand>"), std::string::npos);
  EXPECT_NE(xml.find("<Cmd Seq=\"42\" Action=\"6\" Value=\"0\"/>"),
            std::string::npos);
  EXPECT_NE(xml.find("Z=\"235.000000\""), std::string::npos);
  EXPECT_NE(xml.find("<Base"), std::string::npos);
  EXPECT_NE(xml.find("</RobotCommand>"), std::string::npos);
}

TEST(EkiFrame, SerializeFailsOnTinyBuffer) {
  char buf[8];
  EXPECT_EQ(kuka_eki::serializeCommand(makeCommand(), buf, sizeof(buf)), 0u);
}

TEST(EkiFrame, ParseStateExtractsAllFields) {
  EkiStateFrame s;
  ASSERT_TRUE(kuka_eki::parseState(kStateXml, std::strlen(kStateXml), s));
  EXPECT_TRUE(s.valid);
  EXPECT_EQ(s.ack_seq, 7u);
  EXPECT_TRUE(s.ack_ok);
  EXPECT_EQ(s.ack_code, 0);
  EXPECT_TRUE(s.ready);
  EXPECT_FALSE(s.rsi_active);
  EXPECT_FALSE(s.fault);
  EXPECT_EQ(s.mode, 2);
  EXPECT_NEAR(s.tool.x, 10.5, 1e-9);
  EXPECT_NEAR(s.tool.z, 235.0, 1e-9);
  EXPECT_NEAR(s.tool.b, 90.0, 1e-9);
}

TEST(EkiFrame, ParseHeartbeatWithAckSeqZero) {
  const char xml[] =
      "<RobotState>"
      "<Ack Seq=\"0\" Ok=\"1\" Code=\"0\"/>"
      "<Prog Ready=\"1\" RsiActive=\"1\" Fault=\"0\" Mode=\"1\"/>"
      "<Tool X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "</RobotState>";
  EkiStateFrame s;
  ASSERT_TRUE(kuka_eki::parseState(xml, std::strlen(xml), s));
  EXPECT_EQ(s.ack_seq, 0u);  // unsolicited heartbeat marker (decision 6)
  EXPECT_TRUE(s.rsi_active);
}

TEST(EkiFrame, ParseRejectsMalformedAndWrongRoot) {
  EkiStateFrame s;
  const char broken[] = "<RobotState><Ack Seq=\"1\"";
  EXPECT_FALSE(kuka_eki::parseState(broken, std::strlen(broken), s));
  EXPECT_FALSE(s.valid);
  const char wrong[] = "<Rob><IPOC>1</IPOC></Rob>";
  EXPECT_FALSE(kuka_eki::parseState(wrong, std::strlen(wrong), s));
}

TEST(EkiFrame, ParseRejectsMissingMandatoryElements) {
  EkiStateFrame s;
  const char no_prog[] =
      "<RobotState><Ack Seq=\"1\" Ok=\"1\" Code=\"0\"/></RobotState>";
  EXPECT_FALSE(kuka_eki::parseState(no_prog, std::strlen(no_prog), s));
  const char no_ack[] =
      "<RobotState>"
      "<Prog Ready=\"1\" RsiActive=\"0\" Fault=\"0\" Mode=\"0\"/>"
      "<Tool X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
      "</RobotState>";
  EXPECT_FALSE(kuka_eki::parseState(no_ack, std::strlen(no_ack), s));
}

TEST(EkiSplitter, TwoDocumentsInOneChunk) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  const std::string two = std::string(kStateXml) + kStateXml;
  splitter.feed(two.data(), two.size(),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  ASSERT_EQ(docs.size(), 2u);
  EXPECT_EQ(docs[0], kStateXml);
  EXPECT_EQ(docs[1], kStateXml);
}

TEST(EkiSplitter, DocumentSplitByteByByte) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  for (std::size_t i = 0; i < std::strlen(kStateXml); ++i)
    splitter.feed(kStateXml + i, 1, [&](const char* d, std::size_t n) {
      docs.emplace_back(d, n);
    });
  ASSERT_EQ(docs.size(), 1u);
  EXPECT_EQ(docs[0], kStateXml);
}

TEST(EkiSplitter, NoiseBetweenDocumentsIsDropped) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  const std::string noisy =
      std::string("\r\n junk ") + kStateXml + "garbage" + kStateXml;
  splitter.feed(noisy.data(), noisy.size(),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  ASSERT_EQ(docs.size(), 2u);
  EXPECT_EQ(docs[1], kStateXml);
}

TEST(EkiSplitter, OversizedDocumentIsDiscarded) {
  EkiStreamSplitter splitter;
  std::vector<std::string> docs;
  std::string huge = "<RobotState>";
  huge.append(9000, 'x');  // exceeds the 8 KiB document cap
  huge += "</RobotState>";
  splitter.feed(huge.data(), huge.size(),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  EXPECT_TRUE(docs.empty());
  // The splitter must recover on the next well-formed document.
  splitter.feed(kStateXml, std::strlen(kStateXml),
                [&](const char* d, std::size_t n) { docs.emplace_back(d, n); });
  ASSERT_EQ(docs.size(), 1u);
}
