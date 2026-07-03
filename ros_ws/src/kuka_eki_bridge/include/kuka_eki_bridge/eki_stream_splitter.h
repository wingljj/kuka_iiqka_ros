#pragma once

#include <cstddef>
#include <functional>
#include <string>

namespace kuka_eki {

// Reassembles complete <RobotState>...</RobotState> documents from the
// TCP byte stream (EKI_Send has no message framing on the wire). Bytes
// before a document start are dropped as noise; a document larger than
// kMaxDoc is discarded and scanning resumes at the next start tag.
// Management channel, not realtime: std::string buffering is fine.
class EkiStreamSplitter {
 public:
  using Sink = std::function<void(const char* data, std::size_t len)>;

  void feed(const char* data, std::size_t len, const Sink& sink);
  void reset() { buffer_.clear(); }

 private:
  static constexpr std::size_t kMaxDoc = 8192;
  static constexpr const char* kStartTag = "<RobotState";
  static constexpr const char* kEndTag = "</RobotState>";

  std::string buffer_;
};

}  // namespace kuka_eki
