# data/

Local data that is **not pushed to Git** (see `.gitignore`):

- `bags/`: rosbags recorded with `scripts/record_bag.sh`.
- `runs/`: results you want to keep. By default `telemetry` and `run_scenario` write to `output_dir` (`/tmp/hk_runs` in `simulation.yaml`).

If a bag matters to the team (e.g. the first real flight), upload it to shared storage and link it here with the date, scenario and commit.

## Shared results

`examples/` contains selected, versioned exports for the team.
See [the CF2 simulation result](examples/README.md) and the
[synthetic cooperative route](examples/cooperative_polygon/); the files in `runs/`
stay local.
