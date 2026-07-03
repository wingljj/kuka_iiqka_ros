#pragma once

#include <cstddef>
#include <cstdint>

namespace kuka_eki {

// Application-layer schema of the EKI management channel (decision 6).
// These action codes, element names, and attribute names are the single
// source of truth for the Plan 5 KRL program and EkiConfig.xml templates.
enum class EkiAction : int {
  QUERY_STATE = 0,
  START_RSI = 1,
  STOP_RSI = 2,
  SET_MODE = 3,
  RESET_FAULT = 4,
  GET_TOOL = 5,
  SET_TOOL_BASE = 6,
};

// Ack.Code values reported by the KRL side.
constexpr int kErrOk = 0;
constexpr int kErrNotReady = 1;
constexpr int kErrFaulted = 2;

// Cartesian frame in mm / deg (KUKA A/B/C = Z-Y-X Euler).
struct Frame6 {
  double x{0}, y{0}, z{0}, a{0}, b{0}, c{0};
};

// One ROS -> KRC command document:
// <RobotCommand><Cmd Seq Action Value/><Tool .../><Base .../></RobotCommand>
// Every command carries all elements (unused ones zeroed) to match the
// fixed-structure parsing habits of EthernetKRL configurations.
struct EkiCommand {
  std::uint32_t seq{0};  // starts at 1; 0 is reserved for heartbeats
  EkiAction action{EkiAction::QUERY_STATE};
  int value{0};          // SET_MODE payload; 0 otherwise
  Frame6 tool;
  Frame6 base;
};

// One KRC -> ROS state document:
// <RobotState><Ack Seq Ok Code/><Prog Ready RsiActive Fault Mode/>
//   <Tool X Y Z A B C/></RobotState>
// Ack.Seq == 0 marks an unsolicited heartbeat push.
struct EkiStateFrame {
  std::uint32_t ack_seq{0};
  bool ack_ok{false};
  int ack_code{0};
  bool ready{false};
  bool rsi_active{false};
  bool fault{false};
  int mode{0};
  Frame6 tool;
  bool valid{false};
};

// Serializes a command document into buf (NUL-terminated). Returns the
// payload length (excluding NUL), or 0 if the buffer is too small.
std::size_t serializeCommand(const EkiCommand& cmd, char* buf,
                             std::size_t buf_size);

// Parses one complete RobotState document. Returns false (out.valid =
// false) on malformed XML, wrong root, or missing Ack/Prog/Tool elements
// or any of their mandatory attributes.
bool parseState(const char* data, std::size_t len, EkiStateFrame& out);

}  // namespace kuka_eki
