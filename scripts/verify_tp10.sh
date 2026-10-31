#!/usr/bin/env bash
# Pruebas del simulador propio; requiere scripts/build.sh ejecutado previamente.
set -eo pipefail
TP10_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source "$TP10_ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/tp10-ros-logs}"
# Un dominio de pruebas sin nodos externos evita mezclar comandos y servicios.
/usr/bin/python3 -m pytest "$TP10_ROOT/ros2_ws/src/simulation_adapter/test" -q "$@"
