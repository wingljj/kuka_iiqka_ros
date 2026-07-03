# EKI management-channel state published by kuka_eki_bridge on
# /kuka/eki/state (spec sections 5.2, 6.2). Tool frame in mm / deg
# (KUKA A/B/C = Z-Y-X Euler); tool fields are meaningful once
# state_fresh is true (they mirror the last RobotState heartbeat).
Header header
bool connected              # TCP link to the KRC EKI server is up
bool state_fresh            # a RobotState frame arrived within state_timeout_s
bool program_ready          # KRL program online and ready (spec 6.2)
bool rsi_active             # RSI context active on the KRC
bool fault                  # KRC-side non-realtime fault latched
uint8 mode                  # operating mode reported by the KRL program
uint32 reconnects           # completed reconnect cycles after the first connect
float64 state_age           # s since the last RobotState frame; -1 before first
float64 tool_x
float64 tool_y
float64 tool_z
float64 tool_a
float64 tool_b
float64 tool_c
