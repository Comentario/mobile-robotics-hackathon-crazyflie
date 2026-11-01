#!/usr/bin/env bash
# Receive the current flight only; never substitute a recording.
set -euo pipefail
for argument in "$@"; do
  if [[ "$argument" == --replay || "$argument" == --replay=* ]]; then
    echo 'Live viewer does not accept recordings.' >&2
    exit 2
  fi
done
exec bash "$(dirname "${BASH_SOURCE[0]}")/cf2_view_rviz.sh" "$@"
