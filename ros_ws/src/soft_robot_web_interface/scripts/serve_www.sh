#!/usr/bin/env bash
# Serves the static frontend with the python3 stdlib server. Shim layer
# so roslaunch can own the process (it appends __name/__log remapping
# args that http.server would choke on — same pattern as the bringup
# run_mock.sh). Usage: serve_www.sh [port]
set -eu
PORT="${1:-8080}"
WWW_DIR="$(cd "$(dirname "$0")/../www" && pwd)"
exec python3 -m http.server "$PORT" --directory "$WWW_DIR" --bind 0.0.0.0
