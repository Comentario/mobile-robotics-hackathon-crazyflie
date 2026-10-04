#!/usr/bin/env bash
# Ejecuta todos los escenarios de simulación con evaluación pass/fail (nivel 1, docs/simulations.md).
# Uso: bash scripts/run_all_scenarios.sh [patrón]   p. ej. "T0*" o "T1[0-2]*"
# Requiere: bash scripts/build.sh && source ros2_ws/install/setup.bash
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATTERN="${1:-T*}"
SCENARIOS="$REPO/ros2_ws/src/hackathon_bringup/config/scenarios"
for f in "$SCENARIOS"/$PATTERN.yaml; do
  name="$(basename "$f" .yaml)"
  echo "=== $name ==="
  timeout 120 ros2 launch hackathon_bringup simulation.launch.py scenario:="$name" evaluate:=true \
    > "/tmp/hk_$name.log" 2>&1 || echo "  (launch terminó con código $?; log en /tmp/hk_$name.log)"
done
echo "Resultados: /tmp/hk_runs/results.csv (output_dir en simulation.yaml)."
