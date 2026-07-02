#pragma once

#include <cstddef>
#include <cstdint>

namespace kuka_rsi {

// One KRC -> PC state frame (spec section 6.1, <Rob Type="KUKA">).
// Units: mm/deg for the Cartesian pose, deg for axis angles.
struct RobFrame {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};  // RIst
  double axis_deg[6] = {0, 0, 0, 0, 0, 0};    // AIPos A1..A6
  double delay{0};                            // Delay D (optional)
  int mode{0};                                // Mode M (optional)
  std::uint64_t ipoc{0};                      // IPOC (mandatory, echoed back)
  bool valid{false};
};

// One PC -> KRC correction frame (spec section 6.1, <Sen Type="ROS">).
// Correction units: mm and deg per RSI cycle.
struct SenFrame {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};  // RKorr
  int stop{0};                                // Stop S: 1 = PC requests stop
  std::uint64_t watchdog{0};                  // Watchdog W: PC liveness counter
  std::uint64_t ipoc{0};                      // echo of the received IPOC
};

// Parses a <Rob> frame. Returns false (and out.valid = false) on malformed
// XML, wrong root element, or missing RIst/AIPos/IPOC or any of their
// mandatory attributes. Note: tinyxml2 allocates internally; this is the
// declared realtime deviation of the RSI read path.
bool parseRobFrame(const char* data, std::size_t len, RobFrame& out);

// Serializes a <Sen> frame into buf (NUL-terminated). Returns the payload
// length (excluding NUL), or 0 if the buffer is too small. Allocation-free.
std::size_t serializeSenFrame(const SenFrame& frame, char* buf,
                              std::size_t buf_size);

}  // namespace kuka_rsi
