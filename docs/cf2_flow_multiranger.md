# Drone 2: Flow Deck + Multi-ranger

**Current update:** at the explicit request of the operator, the `--fly` lock, the
automatic height/telemetry supervision, the flight limits and the manual Enter/Ctrl+C
cut-off added by the script were removed. Automatic recording is kept, with the
`cf2_flight_` prefix. The firmware protections were not modified. Enter no longer cuts the
motors; Ctrl+C behaves as normal in Python and leaving the MotionCommander context tries
to land. The controls and locks mentioned below are historical. The cause of the
unexpected climbs is still pending.

**Status at the time: flight locked again.** In `cf2_guarded_hover_a56d393b7832.jsonl`, a
hover at 0.3 m was requested with a subsequent 0.2 m advance, and a climb happened during
the hover window: estimated z 0.206 → 0.513 m between t=4.3 and 4.7 s from the first log;
bottom range 168 → 498 mm. The height stop triggered, before the advance. The height then
reached about 0.595 m (bottom range 598 mm). The operator reports a flip on impact. It was
not a controlled landing. This log does not validate the advance and must not be used as a
map of the flight. The cause of the climb is still undetermined. Review the vertical control
and the real firmware configuration with the mentor before enabling flights again.
The following flight instructions describe historical rehearsals.

**Incident (10 October 2026):** the operator reports a climb to the ceiling when the
landing of the left test at 1 m started. The console log only contains lateral ranges; it
does not allow the cause to be determined. After the mentor's authorization, `--fly` only
allows diagnosis at 0.3 m, for up to 3 seconds, with automatic recording and an interactive
terminal. The historical commands at 1 m stay locked. Inspect the hardware after the
contact and diagnose pose, bottom range and estimator/controller parameters with the motors
disarmed. The cause is still pending; the next flight requires the mentor's supervision and
a prior test of the stop.

### Authorized diagnostic test

Close cfclient and Crazyswarm2 before using the direct script. The client button and the
CF Adapter `/hk/emergency` require their own connections; do not use simultaneous
connections as the script's emergency mechanism. First test `--test-emergency` with the
drone resting: it requests a stop without arming. Check the locked state in the client and
restart afterwards.

The short flight runs with `--fly --height 0.3 --hover-seconds 3`. Enter or Ctrl+C in the
script terminal requests a motor cut, not a landing. The supervision uses monotonic time and
cuts on missing pose/range telemetry for more than 1 s, estimated height or bottom range
above 0.5 m, invalid bottom range, or an up return below 0.3 m. It is supervision from the
laptop: the stop depends on the link, has no physical confirmation and can cause a fall. It
is not a guarantee and does not solve the cause of the incident. Telemetry is kept during
the landing and for one second afterwards.

Hover `cf2_guarded_hover_bff901ea20d3.jsonl`: maximum height 0.327 m; final descent down to
a bottom range of 9–27 mm, with no emergency stop recorded. This provides evidence of the
descent; the cause of the 1 m incident is still pending. The map of that window was
generated with `--hover-only --min-height 0.2 --max-height 0.4`.

Next minimal flight: add `--forward 0.2` to the short flight. It advances at 0.1 m/s, waits
1 s and lands at the new point, without returning to the origin. The front area must be
cleared by the operator: a range without return does not prove that it is clear. The
advance is skipped if the initial front return is less than or equal to the requested
distance + 0.35 m; during the advance, a front return below 0.35 m requests a stop. From the
start of the advance, a maximum XY displacement of 0.6 m according to the estimator is
supervised. These are diagnostic limits, not autonomous navigation nor a braking guarantee.
Keep obstacles still.

Priority: telemetry → short hover → controlled flight → local 2D map. Communication with
drone 1 comes later. Lighthouse is not required for the hover test. The Multi-ranger
measures obstacles; Flow/the estimator provides local motion. This combination does not
guarantee a drift-free global position.

## Official basis

- https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
- https://www.bitcraze.io/documentation/tutorials/getting-started-with-stem-drone-bundle/
- https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/step-by-step/sbs_motion_commander.py

cflib prepared from commit `3c1cb0d024fef6f45d08f0ad236a22ac395dd924`. Our script adapts
the official APIs and requires an explicit URI. Without `--fly` it only connects and reads
the sensors. Deck and log detection is validated on hardware.

## Running it

First confirm the drone 2 URI, USB/Crazyradio access, battery, mounting and firmware
compatible with the official example. Close any client or server controlling that same
drone. Prepare a clear area with a textured floor.

Terminal 1, telemetry (replace the URI):

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ..
.venv-cf2/bin/python scripts/cf2_first_hover.py --uri 'radio://RADIO/CHANNEL/RATE/ADDRESS'
```

After validating the pose and the six ranges, the same command with `--fly` requests a
takeoff to 0.3 m, a 3 second hover and a landing. Validated by the operator. It is a basic
test, without automatic avoidance nor a landing guarantee if the link is lost. Do not start
the repository's full mission for it.

## Later map

### Hover with one sensor reading

`--fly --height 1.0 --hover-seconds 10 --sensor front` requests a climb to 1 m, a
highlighted front reading during 10 seconds of hover and a landing. `--sensor` also accepts
`left`, `back` and `right`, relative to the drone body. All channels are still recorded if
`--record` is added; every row includes `hovering` to identify the measurement window.
Without these options the original hover at 0.3 m for 3 seconds is kept.
The 1 m flight still requires physical validation. Keep people and objects out of reach of
the propellers; use a stand to measure nearby obstacles. The script does not avoid
obstacles automatically.

The next test is **without flight**. Close cfclient and record for 30 seconds:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ..
.venv-cf2/bin/python scripts/cf2_first_hover.py --uri 'radio://0/80/2M/E7E7E7E743' --seconds 30 --record data/runs/cf2_sensor_check_01.jsonl
```

With the drone resting, hold a flat card in front of each lateral sensor, one at a time, at
20 cm and then at 50 cm. Keep every position for a few seconds. We expect about 200 and
500 mm on the corresponding ray. Readings without return do not, by themselves, prove that
the space is free. This test validates the sensors; the cards will appear as accumulated
obstacles in the test map.

```bash
.venv-cf2/bin/python scripts/cf2_build_map.py data/runs/cf2_sensor_check_01.jsonl --output data/runs/cf2_sensor_map_01
```

It produces `map.pgm`, `map.yaml` and `map.json`: 5 cm resolution, 8 × 8 m local area,
grey unknown, white free, black occupied. The recording file and the output folder must be
new to avoid overwriting previous results. For later hover recordings, use
`--min-height 0.2 --max-height 0.4` in the generator to exclude ground, takeoff and landing
samples.

The generator uses the yaw in degrees and converts millimetres to metres. It pairs ranges
with the closest pose within 50 ms; it discards returns outside 50–4000 mm, including
32766. It uses the drone centre as the approximate origin of every ray. It accumulates
obstacles without erasing them: moved objects leave marks. It performs no SLAM nor drift
correction; the map does not yet enable autonomous navigation.

Store the XY/yaw pose and the front/back/left/right rays with timestamps. Transform the rays
into the local frame, mark free space up to the return and occupancy at the end; handle
invalid and no-return ranges explicitly. Keep a 2D grid at an approximately constant
height. That map represents a horizontal slice, not the full volume. Before autonomous
navigation, validate drift, temporal consistency, braking, obstacles and flight limits.

Sharing data with drone 1 requires aligning the frames and measuring the uncertainty;
sending topics between drones does not make their coordinates match.

## Status

Branch `feat/cf2-flow-multiranger`. Rover work removed. First basic flight validated on
10 October 2026 from the WSL of Person 3: the operator confirms a satisfactory climb, hover
and landing. The telemetry shows a height close to 0.3 m and the descent. The log ends
before the final contact; the landing confirmation comes from the operator. This validates
neither navigation nor the 2D map.

Working URI: `radio://0/80/2M/E7E7E7E743`. Channel 43 not validated: it failed with
`Too many packets lost`. Crazyradio 2.0 attached with usbipd; plugdev permissions
configured. `bcFlow2=1`, `bcZRanger2=1`, `bcMultiranger=1`. The Flow Deck was not
discovered while it was mounted upside down; fixing the mounting solved the detection.
The back sensor persistently stopped reporting millimetres; every ray still has to be
validated with objects at known distances before mapping.

The requested integration branch was not in the local references; fetching required
authentication.

## Client and radio configuration in WSL

Install `libxcb-cursor0` and `usbutils` in Ubuntu. In Windows, use usbipd-win to share and
attach to WSL the Crazyflie connected over a data USB cable and then the Crazyradio.
`usbipd list` identifies the BUSID; `usbipd bind --busid BUSID` requires an administrator
PowerShell; `usbipd attach --wsl --busid BUSID` attaches it. Check `lsusb` and the
permissions before connecting with the client.

Open the client installed in the isolated environment:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/cfclient
```

Connect through `usb://0`, open Connect → Configure 2.x and write down the current values.
Agree on a channel with the team and use an exclusive address. Write stores the
configuration; restart and scan with the new address. The Crazyradio index, the RF channel
and the drone address are different concepts. The address was set to `E7E7E7E743`; the
working channel observed is 80.

Sources: https://learn.microsoft.com/en-us/windows/wsl/connect-usb and
https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/
