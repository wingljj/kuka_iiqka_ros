#include "kuka_rsi_hw_interface/rsi_mock_core.h"

#include <tinyxml2.h>

#include <cinttypes>
#include <cstdio>

namespace kuka_rsi {

RsiMockCore::RsiMockCore(const MockConfig& cfg)
    : x_(cfg.x0), y_(cfg.y0), z_(cfg.z0), a_(cfg.a0), b_(cfg.b0), c_(cfg.c0),
      next_ipoc_(cfg.ipoc_start), ipoc_step_(cfg.ipoc_step) {}

std::size_t RsiMockCore::buildStateFrame(char* buf, std::size_t size) {
  const int n = std::snprintf(
      buf, size,
      "<Rob Type=\"KUKA\">"
      "<RIst X=\"%.4f\" Y=\"%.4f\" Z=\"%.4f\" A=\"%.4f\" B=\"%.4f\" "
      "C=\"%.4f\"/>"
      "<AIPos A1=\"0.0\" A2=\"0.0\" A3=\"0.0\" A4=\"0.0\" A5=\"0.0\" "
      "A6=\"0.0\"/>"
      "<Delay D=\"0\"/>"
      "<Mode M=\"1\"/>"
      "<IPOC>%" PRIu64 "</IPOC>"
      "</Rob>",
      x_, y_, z_, a_, b_, c_, next_ipoc_);
  if (n <= 0 || static_cast<std::size_t>(n) >= size) return 0;
  awaited_ipoc_ = next_ipoc_;
  awaiting_ = true;
  next_ipoc_ += ipoc_step_;
  ++stats_.frames_sent;
  return static_cast<std::size_t>(n);
}

bool RsiMockCore::applyReply(const char* data, std::size_t len) {
  tinyxml2::XMLDocument doc;
  if (data == nullptr || doc.Parse(data, len) != tinyxml2::XML_SUCCESS) {
    ++stats_.parse_errors;
    return false;
  }
  const tinyxml2::XMLElement* sen = doc.FirstChildElement("Sen");
  if (sen == nullptr) {
    ++stats_.parse_errors;
    return false;
  }
  const tinyxml2::XMLElement* rkorr = sen->FirstChildElement("RKorr");
  const tinyxml2::XMLElement* ipoc = sen->FirstChildElement("IPOC");
  if (rkorr == nullptr || ipoc == nullptr || ipoc->GetText() == nullptr) {
    ++stats_.parse_errors;
    return false;
  }
  std::uint64_t echoed = 0;
  if (std::sscanf(ipoc->GetText(), "%" SCNu64, &echoed) != 1) {
    ++stats_.parse_errors;
    return false;
  }
  ++stats_.replies_received;
  if (!awaiting_ || echoed != awaited_ipoc_) {
    ++stats_.ipoc_echo_errors;
    return false;
  }
  awaiting_ = false;

  // Relative Cartesian correction: integrate into the pose (spec 6.1).
  x_ += rkorr->DoubleAttribute("X");
  y_ += rkorr->DoubleAttribute("Y");
  z_ += rkorr->DoubleAttribute("Z");
  a_ += rkorr->DoubleAttribute("A");
  b_ += rkorr->DoubleAttribute("B");
  c_ += rkorr->DoubleAttribute("C");

  const tinyxml2::XMLElement* stop = sen->FirstChildElement("Stop");
  if (stop != nullptr) stats_.last_stop = stop->IntAttribute("S");
  const tinyxml2::XMLElement* wd = sen->FirstChildElement("Watchdog");
  if (wd != nullptr) {
    // Int64Attribute: tinyxml2 6.2 has no Unsigned64Attribute (7.0+).
    stats_.last_watchdog =
        static_cast<std::uint64_t>(wd->Int64Attribute("W"));
  }
  return true;
}

}  // namespace kuka_rsi
