#!/usr/bin/env bash
# Boot-time calibrated-payload loader (Task 8 Step 2 fallback).
#
# roslaunch cannot conditionally load an optional file: $(eval ...) rejects
# any double-underscore expression and its builtins whitelist (list/dict/
# map/str/float/int) has no file-existence primitive, so the planned
# if="$(eval __import__('os')...)" attribute fails the static check.
#
# Used as launch-prefix of the soft_robot_manager node: it loads
# payload.yaml onto the parameter server IF the file exists, then execs
# the node. Ordering is deterministic: the parameters are set before the
# manager process starts, hence before its controller-preload thread can
# load_controller (ros_control reads payload/* at controller load time).
set -e
payload="$(rospack find soft_robot_bringup)/config/payload.yaml"
if [ -f "${payload}" ]; then
  rosparam load "${payload}"
fi
exec "$@"
