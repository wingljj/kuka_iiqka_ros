#!/usr/bin/env bash
# Mock launcher shim (Task 8 Step 3 deviation, recorded in the report).
#
# roslaunch appends ROS remapping arguments (__name:=..., __log:=...) to
# every <node> it spawns. The plain, non-roscpp mock executables
# (sri_mock_server / eki_mock_server / kuka_rsi_sim_server) reject any
# unknown argument with a usage message and exit. This shim strips all
# remapping-style arguments (anything containing ":=") and execs the
# requested executable of the requested package.
#
# Usage (as a <node> type): args="<package> <executable> [flags...]"
set -e
pkg="$1"
exe="$2"
shift 2
args=()
for a in "$@"; do
  case "$a" in
    *:=*) ;;  # drop ROS remapping args injected by roslaunch
    *) args+=("$a") ;;
  esac
done
path="$(catkin_find --first-only --libexec "${pkg}" "${exe}")"
exec "${path}" "${args[@]}"
