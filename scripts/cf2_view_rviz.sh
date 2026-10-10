#!/usr/bin/env bash
# Live/replay presentation only. Never opens a radio or sends flight commands.
set -euo pipefail
CF2_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$CF2_REPO/ros2_ws/install/setup.bash"
set -u
cd "$CF2_REPO"
CF2_VIEW_PID=''
cleanup() {
  if [[ -n "$CF2_VIEW_PID" ]]; then
    kill -TERM "$CF2_VIEW_PID" 2>/dev/null || true
    wait "$CF2_VIEW_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
.venv-cf2/bin/python scripts/cf2_live_map.py --ros "$@" &
CF2_VIEW_PID=$!
rviz2 -d "$CF2_REPO/ros2_ws/src/hackathon_bringup/rviz/cf2_live.rviz"
