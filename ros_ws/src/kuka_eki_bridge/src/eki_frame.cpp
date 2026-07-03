#include "kuka_eki_bridge/eki_frame.h"

#include <tinyxml2.h>

#include <cstdio>

namespace kuka_eki {

std::size_t serializeCommand(const EkiCommand& cmd, char* buf,
                             std::size_t buf_size) {
  const int n = std::snprintf(
      buf, buf_size,
      "<RobotCommand>"
      "<Cmd Seq=\"%u\" Action=\"%d\" Value=\"%d\"/>"
      "<Tool X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\"/>"
      "<Base X=\"%f\" Y=\"%f\" Z=\"%f\" A=\"%f\" B=\"%f\" C=\"%f\"/>"
      "</RobotCommand>",
      cmd.seq, static_cast<int>(cmd.action), cmd.value, cmd.tool.x,
      cmd.tool.y, cmd.tool.z, cmd.tool.a, cmd.tool.b, cmd.tool.c, cmd.base.x,
      cmd.base.y, cmd.base.z, cmd.base.a, cmd.base.b, cmd.base.c);
  if (n <= 0 || static_cast<std::size_t>(n) >= buf_size) return 0;
  return static_cast<std::size_t>(n);
}

namespace {

bool readFrame6(const tinyxml2::XMLElement* e, Frame6& out) {
  return e->QueryDoubleAttribute("X", &out.x) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("Y", &out.y) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("Z", &out.z) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("A", &out.a) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("B", &out.b) == tinyxml2::XML_SUCCESS &&
         e->QueryDoubleAttribute("C", &out.c) == tinyxml2::XML_SUCCESS;
}

}  // namespace

bool parseState(const char* data, std::size_t len, EkiStateFrame& out) {
  out = EkiStateFrame{};
  tinyxml2::XMLDocument doc;
  if (doc.Parse(data, len) != tinyxml2::XML_SUCCESS) return false;
  const tinyxml2::XMLElement* root = doc.FirstChildElement("RobotState");
  if (root == nullptr) return false;

  const tinyxml2::XMLElement* ack = root->FirstChildElement("Ack");
  const tinyxml2::XMLElement* prog = root->FirstChildElement("Prog");
  const tinyxml2::XMLElement* tool = root->FirstChildElement("Tool");
  if (ack == nullptr || prog == nullptr || tool == nullptr) return false;

  unsigned seq = 0;
  int ok = 0;
  if (ack->QueryUnsignedAttribute("Seq", &seq) != tinyxml2::XML_SUCCESS ||
      ack->QueryIntAttribute("Ok", &ok) != tinyxml2::XML_SUCCESS ||
      ack->QueryIntAttribute("Code", &out.ack_code) != tinyxml2::XML_SUCCESS)
    return false;
  out.ack_seq = seq;
  out.ack_ok = ok != 0;

  int ready = 0, rsi = 0, fault = 0;
  if (prog->QueryIntAttribute("Ready", &ready) != tinyxml2::XML_SUCCESS ||
      prog->QueryIntAttribute("RsiActive", &rsi) != tinyxml2::XML_SUCCESS ||
      prog->QueryIntAttribute("Fault", &fault) != tinyxml2::XML_SUCCESS ||
      prog->QueryIntAttribute("Mode", &out.mode) != tinyxml2::XML_SUCCESS)
    return false;
  out.ready = ready != 0;
  out.rsi_active = rsi != 0;
  out.fault = fault != 0;

  if (!readFrame6(tool, out.tool)) return false;
  out.valid = true;
  return true;
}

}  // namespace kuka_eki
