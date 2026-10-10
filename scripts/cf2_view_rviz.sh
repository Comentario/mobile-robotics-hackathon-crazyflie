#!/usr/bin/env bash
# Live/replay presentation only. Never opens a radio or sends flight commands.
set -euo pipefail
CF2_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$CF2_REPO/ros2_ws/install/setup.bash"
set -u
cd "$CF2_REPO"
# Keep this viewer separate from older RViz/replay publishers.
export ROS_DOMAIN_ID="${CF2_RVIZ_DOMAIN_ID:-43}"
echo "CF2 viewer | ROS domain $ROS_DOMAIN_ID"
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
sleep 1
if ! kill -0 "$CF2_VIEW_PID" 2>/dev/null; then
  echo 'Mapper failed to start. Close the previous mapper or inspect the error above.' >&2
  exit 1
fi
# Software rendering avoids WSLg GPU/shader problems; set CF2_RVIZ_SOFTWARE=0 to use GPU.
LIBGL_ALWAYS_SOFTWARE="${CF2_RVIZ_SOFTWARE:-1}" QT_QPA_PLATFORM=xcb rviz2 -d "$CF2_REPO/ros2_ws/src/hackathon_bringup/rviz/cf2_live.rviz"
