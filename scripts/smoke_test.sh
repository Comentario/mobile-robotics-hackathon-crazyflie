#!/usr/bin/env bash
# TP-06 smoke test: starts simulation.launch.py and checks that the simulator pose
# reaches /hk/state through state_monitor (simulation_adapter -> /hk/adapter/odom -> /hk/state).
# Usage: bash scripts/smoke_test.sh   (requires bash scripts/build.sh)
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u  # the ROS setup.bash uses undefined variables
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
set -u

LOG=/tmp/hk_smoke_test.log
# set -m: the launch runs in its own process group and does not ignore SIGINT, so it closes with all its nodes.
set -m
ros2 launch hackathon_bringup simulation.launch.py > "$LOG" 2>&1 &
LAUNCH_PID=$!
set +m
trap 'kill -INT -- -$LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null' EXIT

ok=0
for topic in /hk/adapter/odom /hk/state; do
  if msg="$(timeout 15 ros2 topic echo --once "$topic" nav_msgs/msg/Odometry 2>/dev/null)"; then
    pos="$(echo "$msg" | grep -A3 '^    position:' | tail -3 | tr -s ' \n' ' ')"
    echo "OK    $topic  ($pos)"
  else
    echo "FAIL  $topic  no messages within 15 s (log: $LOG)"
    ok=1
  fi
done
exit $ok
