#!/usr/bin/env bash
# Ideal simulation + live RViz. No radio access. Close RViz to stop both.
set -euo pipefail
CF2_SIM_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$CF2_SIM_REPO"
CF2_SIM_VIEW_PID=''
CF2_SIM_RUN_PID=''
cleanup() {
  for pid in "$CF2_SIM_RUN_PID" "$CF2_SIM_VIEW_PID"; do
    if [[ -n "$pid" ]]; then kill -TERM "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash scripts/cf2_view_rviz.sh &
CF2_SIM_VIEW_PID=$!
sleep 2
if ! kill -0 "$CF2_SIM_VIEW_PID" 2>/dev/null; then
  echo 'Viewer failed to start. Inspect the output above.' >&2
  exit 1
fi
.venv-cf2/bin/python scripts/cf2_survey_sim.py "$@" &
CF2_SIM_RUN_PID=$!
wait "$CF2_SIM_VIEW_PID"
