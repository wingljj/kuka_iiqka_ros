// Standalone SRI sensor mock for manual smoke tests and Plan 5 bringup.
// Continuous waveform (optional sine on Fz) and periodic fault injection.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "sri_force_torque_driver/sri_mock_server.h"

namespace {

// Wraps the library mock for continuous waveforms; uses rate 0 + explicit
// sendFrames pacing so the waveform can evolve per frame. Listens on
// --port when given, else on a kernel-chosen port (both printed below).
// (Plan 4 follow-up 2: the old comment claimed a fixed port was required.)
struct Options {
  double rate_hz = 250.0;
  double fz = 0.0;
  double sine_amp = 0.0;
  double sine_hz = 0.0;
  int bad_every = 0;
  int port = 0;
};

int usage(int code) {
  std::printf(
      "usage: sri_mock_server [--port N] [--rate HZ] [--fz N] [--sine-amp N]\n"
      "                       [--sine-hz HZ] [--bad-every N]\n"
      "Listens on 127.0.0.1:--port (0/default = kernel-chosen, printed).\n");
  return code;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has_val = i + 1 < argc;
    if (a == "--help" || a == "-h") return usage(0);
    if (!has_val) return usage(1);
    const double v = std::atof(argv[++i]);
    if (a == "--rate") opt.rate_hz = v;
    else if (a == "--fz") opt.fz = v;
    else if (a == "--sine-amp") opt.sine_amp = v;
    else if (a == "--sine-hz") opt.sine_hz = v;
    else if (a == "--bad-every") opt.bad_every = static_cast<int>(v);
    else if (a == "--port") opt.port = static_cast<int>(v);
    else return usage(1);
  }
  sri::SriMockConfig cfg;
  cfg.listen_port = static_cast<std::uint16_t>(opt.port);
  cfg.require_start_command = true;
  cfg.rate_hz = 0.0;  // paced manually below so the waveform can evolve
  sri::SriMockServer mock(cfg);
  if (!mock.start()) {
    std::fprintf(stderr, "bind/listen failed\n");
    return 1;
  }
  std::printf("sri_mock_server listening on 127.0.0.1:%u (rate %.0f Hz)\n",
              mock.port(), opt.rate_hz);
  std::fflush(stdout);
  const auto period = std::chrono::microseconds(
      static_cast<long>(1e6 / (opt.rate_hz > 0 ? opt.rate_hz : 250.0)));
  double t = 0.0;
  long frame_no = 0;
  for (;;) {
    const double fz =
        opt.fz + opt.sine_amp * std::sin(2.0 * M_PI * opt.sine_hz * t);
    mock.setWrench(0.0f, 0.0f, static_cast<float>(fz), 0.0f, 0.0f, 0.0f);
    ++frame_no;
    if (opt.bad_every > 0 && frame_no % opt.bad_every == 0) {
      mock.sendBadChecksumFrame();
    } else {
      mock.sendFrames(1);
    }
    t += 1e-6 * static_cast<double>(period.count());
    std::this_thread::sleep_for(period);
  }
  return 0;
}
