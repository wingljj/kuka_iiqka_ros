# Current control mode, profile, and system state (spec sections 10-11).
uint8 SYSTEM_OFFLINE=0
uint8 SYSTEM_CONNECTED=1
uint8 SYSTEM_READY=2
uint8 SYSTEM_SERVOING=3
uint8 SYSTEM_CALIBRATING=4
uint8 SYSTEM_DEGRADED=5
uint8 SYSTEM_FAULT=6
Header header
uint8 mode
uint8 profile
uint8 system_state
