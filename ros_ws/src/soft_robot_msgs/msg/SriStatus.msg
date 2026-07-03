# SRI force/torque sensor link diagnostics published by
# sri_force_torque_driver on /sri_ft/status (spec section 5.6).
# Counters are cumulative since node start.
Header header
bool connected              # TCP link to the sensor box is up
bool streaming              # a valid frame arrived within sample_timeout_s
uint32 reconnects           # completed reconnect cycles after the first connect
uint64 samples              # decoded valid frames
uint64 bad_frames           # checksum/length failures
uint64 package_gaps         # non-consecutive package-number events
uint32 zero_rejects         # tare captures rejected by bias_limit_n
float64 last_sample_age     # s since the last valid sample; -1 before the first
bool zero_active            # a tare capture is currently averaging samples
float64 filter_cutoff_hz    # driver-side low-pass; 0 = raw passthrough
