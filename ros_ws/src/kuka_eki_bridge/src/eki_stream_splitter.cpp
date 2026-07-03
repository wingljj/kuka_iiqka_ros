#include "kuka_eki_bridge/eki_stream_splitter.h"

#include <cstring>

namespace kuka_eki {

// C++14: odr-used constexpr static members need namespace-scope definitions.
constexpr std::size_t EkiStreamSplitter::kMaxDoc;
constexpr const char* EkiStreamSplitter::kStartTag;
constexpr const char* EkiStreamSplitter::kEndTag;

void EkiStreamSplitter::feed(const char* data, std::size_t len,
                             const Sink& sink) {
  buffer_.append(data, len);
  for (;;) {
    const std::size_t start = buffer_.find(kStartTag);
    if (start == std::string::npos) {
      // Keep a tail shorter than the start tag: it may be a split prefix.
      const std::size_t tag_len = std::strlen(kStartTag);
      if (buffer_.size() >= tag_len)
        buffer_.erase(0, buffer_.size() - (tag_len - 1));
      return;
    }
    if (start > 0) buffer_.erase(0, start);  // drop leading noise
    const std::size_t end = buffer_.find(kEndTag);
    if (end == std::string::npos) {
      if (buffer_.size() > kMaxDoc) {
        // Oversized document: discard the start tag and rescan.
        buffer_.erase(0, std::strlen(kStartTag));
        continue;
      }
      return;  // incomplete document, wait for more bytes
    }
    const std::size_t doc_len = end + std::strlen(kEndTag);
    if (doc_len <= kMaxDoc) sink(buffer_.data(), doc_len);
    buffer_.erase(0, doc_len);
  }
}

}  // namespace kuka_eki
