// Standalone KRC-side RSI mock (spec section 15.2). Sends state frames at
// the configured cycle to the PC-side hardware interface and validates the
// returned RKorr/IPOC echo. Test/commissioning tool: not realtime code.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "kuka_rsi_hw_interface/rsi_mock_core.h"
#include "kuka_rsi_hw_interface/rsi_mock_server.h"
#include "kuka_rsi_hw_interface/udp_transport.h"

namespace {

struct Options {
  std::string target_ip{"127.0.0.1"};
  int target_port{49152};
  int cycle_ms{4};
  int reply_timeout_ms{4};
  kuka_rsi::MockConfig mock;
};

bool parseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    if (arg == "--help") return false;
    if (!has_value) return false;
    const char* v = argv[++i];
    if (arg == "--target-ip") opt.target_ip = v;
    else if (arg == "--target-port") opt.target_port = std::atoi(v);
    else if (arg == "--cycle-ms") opt.cycle_ms = std::atoi(v);
    else if (arg == "--reply-timeout-ms") opt.reply_timeout_ms = std::atoi(v);
    else if (arg == "--x0") opt.mock.x0 = std::atof(v);
    else if (arg == "--y0") opt.mock.y0 = std::atof(v);
    else if (arg == "--z0") opt.mock.z0 = std::atof(v);
    else if (arg == "--a0") opt.mock.a0 = std::atof(v);
    else if (arg == "--b0") opt.mock.b0 = std::atof(v);
    else if (arg == "--c0") opt.mock.c0 = std::atof(v);
    else return false;
  }
  return opt.target_port > 0 && opt.cycle_ms > 0 && opt.reply_timeout_ms > 0;
}

void printUsage() {
  std::printf(
      "Usage: kuka_rsi_sim_server [--target-ip IP] [--target-port PORT]\n"
      "  [--cycle-ms N] [--reply-timeout-ms N]\n"
      "  [--x0 V] [--y0 V] [--z0 V] [--a0 V] [--b0 V] [--c0 V]\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    printUsage();
    return 1;
  }

  kuka_rsi::RsiMockCore core(opt.mock);
  kuka_rsi::UdpTransport udp;
  if (!udp.bind("0.0.0.0", 0)) {
    std::fprintf(stderr, "kuka_rsi_sim_server: failed to bind UDP socket\n");
    return 1;
  }
  std::printf("kuka_rsi_sim_server: -> %s:%d, cycle %d ms\n",
              opt.target_ip.c_str(), opt.target_port, opt.cycle_ms);

  char tx[1024];
  std::uint64_t cycle = 0;
  for (;;) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(opt.cycle_ms);
    const std::size_t n = core.buildStateFrame(tx, sizeof(tx));
    if (n == 0 ||
        !udp.sendTo(opt.target_ip,
                    static_cast<std::uint16_t>(opt.target_port), tx, n)) {
      std::fprintf(stderr, "kuka_rsi_sim_server: send failed\n");
      return 1;
    }
    // Resync receive (Task 8c debt fix): drains any stale backlog left by
    // a PC-side stall inside the window instead of staying one packet
    // behind forever. Records the timeout itself when nothing arrives.
    kuka_rsi::receiveReplyWindow(core, udp, opt.reply_timeout_ms);
    if (++cycle % 250 == 0) {  // once a second at 4 ms
      const kuka_rsi::MockStats& s = core.stats();
      std::printf(
          "sent=%llu ok=%llu tmo=%llu echo_err=%llu parse_err=%llu "
          "stop=%d pose x=%.3f y=%.3f z=%.3f a=%.3f b=%.3f c=%.3f\n",
          static_cast<unsigned long long>(s.frames_sent),
          static_cast<unsigned long long>(s.replies_received),
          static_cast<unsigned long long>(s.reply_timeouts),
          static_cast<unsigned long long>(s.ipoc_echo_errors),
          static_cast<unsigned long long>(s.parse_errors), s.last_stop,
          core.x(), core.y(), core.z(), core.a(), core.b(), core.c());
      std::fflush(stdout);
    }
    std::this_thread::sleep_until(deadline);
  }
  return 0;
}
