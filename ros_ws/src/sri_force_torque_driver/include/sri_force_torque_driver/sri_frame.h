#pragma once

#include <cstddef>
#include <cstdint>

namespace sri {

// ASCII commands of the M8128-style acquisition box (decision 1). CRLF
// terminated; the box answers with an ASCII ACK line that the assembler
// skips while hunting for the binary sync bytes.
inline const char* startStreamCommand() { return "AT+GSD\r\n"; }
inline const char* stopStreamCommand() { return "AT+GSD=STOP\r\n"; }

// One decoded data frame: 6 float32 channels Fx Fy Fz Mx My Mz (N / Nm).
struct FtSample {
  float ch[6] = {0, 0, 0, 0, 0, 0};
  std::uint16_t package_number{0};
};

struct AssemblerStats {
  std::uint64_t frames{0};         // valid frames decoded (incl. dropped)
  std::uint64_t bad_checksum{0};
  std::uint64_t bad_length{0};     // declared length != kPayloadLen
  std::uint64_t dropped{0};        // valid frames beyond the caller's max_out
  std::uint64_t skipped_bytes{0};  // non-sync bytes (ASCII ACKs, garbage)
};

// Incremental parser for the binary stream (decision 1):
//   AA 55 | LEN_H LEN_L (=27) | PN_H PN_L | 24 data bytes (6 x f32 LE) | SUM
// SUM = sum of the 24 data bytes & 0xFF. Byte-oriented FSM with fixed
// storage, allocation-free. A corrupted frame costs at most one frame:
// the FSM drops it, counts it, and rescans for the next sync header.
// Host is assumed little-endian for the float decode (x86-64/ARM64).
class SriFrameAssembler {
 public:
  // Feeds raw bytes; writes decoded samples to out (up to max_out).
  // Returns the number of samples written. Valid frames beyond max_out
  // are counted in stats().dropped so the byte stream stays consistent.
  int feed(const std::uint8_t* data, std::size_t len, FtSample* out,
           int max_out);

  const AssemblerStats& stats() const { return stats_; }
  void reset();  // resync on a new connection; keeps counters

 private:
  static constexpr std::uint8_t kSync1 = 0xAA;
  static constexpr std::uint8_t kSync2 = 0x55;
  static constexpr std::size_t kDataLen = 24;                 // 6 x float32
  static constexpr std::size_t kPayloadLen = 2 + kDataLen + 1;  // PN+data+sum

  enum class State { SYNC1, SYNC2, LEN_HIGH, LEN_LOW, PAYLOAD };

  State state_{State::SYNC1};
  std::uint16_t declared_len_{0};
  std::uint8_t payload_[kPayloadLen];
  std::size_t payload_fill_{0};
  AssemblerStats stats_;
};

}  // namespace sri
