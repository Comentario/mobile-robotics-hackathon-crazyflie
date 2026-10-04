#!/usr/bin/env bash
# Compila el workspace ROS 2 del repo. Uso (desde cualquier carpeta, en WSL/Ubuntu):
#   bash scripts/build.sh [argumentos extra de colcon, p. ej. --packages-select state_monitor]
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u  # setup.bash de ROS usa variables sin definir
source /opt/ros/humble/setup.bash
set -u
cd "$REPO/ros2_ws"
colcon build --symlink-install "$@"
echo "Listo. Ahora: source $REPO/ros2_ws/install/setup.bash"
