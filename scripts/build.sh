#!/usr/bin/env bash
# Builds the repository ROS 2 workspace. Usage (from any folder, in WSL/Ubuntu):
#   bash scripts/build.sh [extra colcon arguments, e.g. --packages-select state_monitor]
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u  # the ROS setup.bash uses undefined variables
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "$ROS_SETUP does not exist; set ROS_DISTRO or source ROS before building." >&2
  exit 1
fi
source "$ROS_SETUP"
set -u
cd "$REPO/ros2_ws"
colcon build --symlink-install "$@"
echo "Done. Now: source $REPO/ros2_ws/install/setup.bash"
