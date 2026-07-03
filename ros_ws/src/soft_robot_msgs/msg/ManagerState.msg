# System-level manager state (spec sections 5.5, 11) published by
# soft_robot_manager on /soft_robot/manager_state. system_state uses the
# ModeState.SYSTEM_* constants (single source for the state numbering).
Header header
uint8 system_state          # ModeState.SYSTEM_* value
uint8 mode                  # active/target ModeCommand.MODE_* while servoing
uint8 profile               # ModeCommand.PROFILE_* selected for servo
bool eki_connected          # /kuka/eki/state: connected && state_fresh
bool eki_program_ready      # KRL program online and ready
bool rsi_connected          # /kuka/rsi/state fresh && connected
bool rsi_fault              # latched RSI communication fault
bool sri_streaming          # /sri_ft/status fresh && streaming
bool tool_synced            # $TOOL read through EKI since (re)connect
bool calibrating            # payload calibration action running
string active_controller    # running controller name; "" when none
