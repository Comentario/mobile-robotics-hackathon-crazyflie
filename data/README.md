# data/

Datos locales que **no se suben a Git** (ver `.gitignore`):

- `bags/`: rosbags grabados con `scripts/record_bag.sh`.
- `runs/`: resultados que quieran conservar. Por defecto `telemetry` y `run_scenario` escriben en `output_dir` (`/tmp/hk_runs` en `simulation.yaml`).

Si un bag es importante para el equipo (p. ej. el primer vuelo real), súbanlo a un almacenamiento compartido y enlácenlo aquí con fecha, escenario y commit.
