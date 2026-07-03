// Standalone KRC-side EKI mock for manual smoke tests and Plan 5 bringup.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "kuka_eki_bridge/eki_mock_server.h"

namespace {
int usage(int code) {
  std::printf(
      "usage: eki_mock_server [--port N] [--heartbeat-ms N] [--start-faulted]\n"
      "Listens on 127.0.0.1:--port (0/default = kernel-chosen, printed).\n");
  return code;
}
}  // namespace

int main(int argc, char** argv) {
  kuka_eki::EkiMockConfig cfg;
  cfg.heartbeat_period_s = 0.1;
  bool start_faulted = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") return usage(0);
    if (a == "--start-faulted") {
      start_faulted = true;
    } else if (a == "--heartbeat-ms" && i + 1 < argc) {
      cfg.heartbeat_period_s = std::atof(argv[++i]) / 1000.0;
    } else if (a == "--port" && i + 1 < argc) {
      cfg.listen_port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
    } else {
      return usage(1);
    }
  }
  kuka_eki::EkiMockServer mock(cfg);
  if (!mock.start()) {
    std::fprintf(stderr, "bind/listen failed\n");
    return 1;
  }
  if (start_faulted) mock.injectFault();
  std::printf("eki_mock_server listening on 127.0.0.1:%u (heartbeat %.0f ms)\n",
              mock.port(), cfg.heartbeat_period_s * 1000.0);
  std::fflush(stdout);
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
