#!/usr/bin/env bash
# Compila el workspace ROS 2 del repo. Uso (desde cualquier carpeta, en WSL/Ubuntu):
#   bash scripts/build.sh [argumentos extra de colcon, p. ej. --packages-select state_monitor]
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u  # setup.bash de ROS usa variables sin definir
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "No existe $ROS_SETUP; define ROS_DISTRO o carga ROS antes de compilar." >&2
  exit 1
fi
source "$ROS_SETUP"
set -u
cd "$REPO/ros2_ws"
colcon build --symlink-install "$@"
echo "Listo. Ahora: source $REPO/ros2_ws/install/setup.bash"
