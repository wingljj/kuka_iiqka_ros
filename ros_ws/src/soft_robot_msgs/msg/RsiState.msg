# Realtime RSI link diagnostics published by kuka_rsi_hw_interface
# (spec section 12.2). Counters are cumulative since node start.
Header header
bool connected                # at least one valid KRC frame received
bool fault                    # latched communication fault (zero output + Stop)
uint64 ipoc                   # last IPOC received from the KRC
uint64 total_timeouts         # read cycles that ended without a frame
uint32 consecutive_timeouts   # current run of missed cycles
uint64 bad_frames             # frames that failed XML validation
uint64 ipoc_jumps             # non-increasing IPOC events
uint64 saturation_count       # write cycles clamped by the hw command limiter
