#!/usr/bin/env bash
# Records the stack topics (TP-40). Usage: bash scripts/record_bag.sh <label>
# Bags go to data/bags/ and are not pushed to Git.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LABEL="${1:-run}"
OUT="$REPO/data/bags/$(date +%Y%m%d_%H%M%S)_$LABEL"
mkdir -p "$REPO/data/bags"
ros2 bag record -o "$OUT" \
  /tf /tf_static \
  /hk/adapter/odom /hk/adapter/ranges \
  /hk/state /hk/localization_health \
  /hk/ranges /hk/ranges/closing_rate \
  /hk/map /hk/goal /hk/path /hk/path/processed \
  /hk/cmd/nav /hk/cmd/safe /hk/replan_request \
  /hk/mission_status /hk/planner_status /hk/safety_state /hk/metrics \
  /hk/sim/ground_truth
