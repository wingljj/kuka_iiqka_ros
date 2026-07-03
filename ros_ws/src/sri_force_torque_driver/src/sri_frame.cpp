#include "sri_force_torque_driver/sri_frame.h"

#include <cstring>

namespace sri {

void SriFrameAssembler::reset() {
  state_ = State::SYNC1;
  declared_len_ = 0;
  payload_fill_ = 0;
}

int SriFrameAssembler::feed(const std::uint8_t* data, std::size_t len,
                            FtSample* out, int max_out) {
  int written = 0;
  for (std::size_t i = 0; i < len; ++i) {
    const std::uint8_t b = data[i];
    switch (state_) {
      case State::SYNC1:
        if (b == kSync1) {
          state_ = State::SYNC2;
        } else {
          ++stats_.skipped_bytes;
        }
        break;
      case State::SYNC2:
        if (b == kSync2) {
          state_ = State::LEN_HIGH;
        } else if (b == kSync1) {
          ++stats_.skipped_bytes;  // previous AA was noise; this one may sync
        } else {
          stats_.skipped_bytes += 2;
          state_ = State::SYNC1;
        }
        break;
      case State::LEN_HIGH:
        declared_len_ = static_cast<std::uint16_t>(b) << 8;
        state_ = State::LEN_LOW;
        break;
      case State::LEN_LOW:
        declared_len_ = static_cast<std::uint16_t>(declared_len_ | b);
        if (declared_len_ != kPayloadLen) {
          ++stats_.bad_length;
          state_ = State::SYNC1;
        } else {
          payload_fill_ = 0;
          state_ = State::PAYLOAD;
        }
        break;
      case State::PAYLOAD:
        payload_[payload_fill_++] = b;
        if (payload_fill_ == kPayloadLen) {
          state_ = State::SYNC1;
          unsigned sum = 0;
          for (std::size_t k = 2; k < 2 + kDataLen; ++k) sum += payload_[k];
          if ((sum & 0xFFu) != payload_[kPayloadLen - 1]) {
            ++stats_.bad_checksum;
            break;
          }
          ++stats_.frames;
          if (written >= max_out) {
            ++stats_.dropped;
            break;
          }
          FtSample& s = out[written++];
          s.package_number = static_cast<std::uint16_t>(
              (static_cast<std::uint16_t>(payload_[0]) << 8) | payload_[1]);
          std::memcpy(s.ch, payload_ + 2, kDataLen);  // little-endian host
        }
        break;
    }
  }
  return written;
}

}  // namespace sri
