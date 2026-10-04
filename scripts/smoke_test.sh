#!/usr/bin/env bash
# Smoke test de TP-06: arranca simulation.launch.py y comprueba que la pose del simulador
# llega a /hk/state pasando por state_monitor (simulation_adapter -> /hk/adapter/odom -> /hk/state).
# Uso: bash scripts/smoke_test.sh   (requiere bash scripts/build.sh)
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u  # setup.bash de ROS usa variables sin definir
source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
set -u

LOG=/tmp/hk_smoke_test.log
# set -m: el launch va en su propio grupo de procesos y no ignora SIGINT, así se cierra con todos sus nodos.
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
    echo "FALLO $topic  sin mensajes en 15 s (log: $LOG)"
    ok=1
  fi
done
exit $ok
