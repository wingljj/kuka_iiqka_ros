#include "kuka_rsi_hw_interface/rsi_frame.h"

#include <tinyxml2.h>

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace kuka_rsi {

namespace {
// Reads all listed attributes as doubles; false if any is missing/invalid.
bool readAttrs(const tinyxml2::XMLElement* e, const char* const* names,
               double* out, int count) {
  for (int i = 0; i < count; ++i) {
    if (e->QueryDoubleAttribute(names[i], &out[i]) != tinyxml2::XML_SUCCESS) {
      return false;
    }
  }
  return true;
}
}  // namespace

bool parseRobFrame(const char* data, std::size_t len, RobFrame& out) {
  out.valid = false;
  if (data == nullptr || len == 0) return false;

  tinyxml2::XMLDocument doc;
  if (doc.Parse(data, len) != tinyxml2::XML_SUCCESS) return false;

  const tinyxml2::XMLElement* rob = doc.FirstChildElement("Rob");
  if (rob == nullptr) return false;

  const tinyxml2::XMLElement* rist = rob->FirstChildElement("RIst");
  const tinyxml2::XMLElement* aipos = rob->FirstChildElement("AIPos");
  const tinyxml2::XMLElement* ipoc = rob->FirstChildElement("IPOC");
  if (rist == nullptr || aipos == nullptr || ipoc == nullptr) return false;

  static const char* const kPose[] = {"X", "Y", "Z", "A", "B", "C"};
  double pose[6];
  if (!readAttrs(rist, kPose, pose, 6)) return false;

  static const char* const kAxes[] = {"A1", "A2", "A3", "A4", "A5", "A6"};
  double axes[6];
  if (!readAttrs(aipos, kAxes, axes, 6)) return false;

  const char* ipoc_text = ipoc->GetText();
  if (ipoc_text == nullptr) return false;
  std::uint64_t ipoc_value = 0;
  if (std::sscanf(ipoc_text, "%" SCNu64, &ipoc_value) != 1) return false;

  out.x = pose[0];
  out.y = pose[1];
  out.z = pose[2];
  out.a = pose[3];
  out.b = pose[4];
  out.c = pose[5];
  for (int i = 0; i < 6; ++i) out.axis_deg[i] = axes[i];
  out.delay = 0;
  out.mode = 0;
  const tinyxml2::XMLElement* delay = rob->FirstChildElement("Delay");
  if (delay != nullptr) delay->QueryDoubleAttribute("D", &out.delay);
  const tinyxml2::XMLElement* mode = rob->FirstChildElement("Mode");
  if (mode != nullptr) mode->QueryIntAttribute("M", &out.mode);
  out.ipoc = ipoc_value;
  out.valid = true;
  return true;
}

}  // namespace kuka_rsi
