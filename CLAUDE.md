# Working in this repo

HIL torque-vectoring race-car simulation in C. Three roles kept in separate
files on purpose (see README.md): the **driver** (`HIL_Firmware/src/motion_control.c`),
the **car** (`HIL_Firmware/src/vehicle_model.c`), and the **ECU**
(`ECU_Firmware/src/torque_vectoring.c`). The ECU only ever sees a `SensorData`
struct — that HIL boundary is enforced by the `ECU_OBJ` build target; don't let
the ECU include anything from `HIL_Firmware/`.

## Evaluating driver / vehicle / controller changes

After ANY change to the motion controller, speed planner, vehicle model, or
tuning gains, run the headless lap evaluator — do not judge a driver change by
eye in the visualiser, and do not rely only on the unit tests (they check
clamping and signs, not whether the car actually makes it round):

```
make eval
```

This runs the full motion-control → ECU → vehicle loop over the FSG 2024 track
as fast as possible (no real-time sleep) and prints lap-tracking metrics. A good
change keeps **off-track ticks at 0** and **a completed lap**, and should not
regress mean/worst cross-track error or lap time. The source is
`tools/eval_lap.c`; the machine-readable summary is the `RESULT ...` line.

Keep evaluation runs at/under **50 s of simulated time** (one lap is ~32 s) —
the evaluator already caps at 50 s. Longer runs waste time without adding
signal.

A `TRACE=1 make eval`-style run (set the env var, then run the eval binary
`HIL_Firmware/build/eval_lap`) prints a per-tick trace (waypoint, curvature,
cross-track error, speed, steer, slip) — use it to localise where the car goes
wide.

### Parameter sweeps

The four highest-leverage tuning gains in `HIL_Firmware/include/motion_control.h`
(`MAX_LATERAL_ACCEL_MS2`, `K_LOOKAHEAD`, `LOOKAHEAD_MIN_M`, `MAX_STEER_RATE_RADS`)
are wrapped in `#ifndef`, so they can be overridden at compile time with `-D`
without editing the header. `tools/sweep.sh` sweeps the first three and prints a
table sorted by clean-lap time. When picking a "fastest" config, only accept one
with **0 off-track ticks** — faster laps that clip apex cones are not valid.

## CI

`.github/workflows/ci.yml` builds, runs `make test`, then runs `make eval` and
posts the lap-evaluation table to the GitHub Actions run summary. CI **fails**
if the car does not complete a lap or goes off-track, so a driver regression is
caught automatically.

## Build note (Windows)

The Makefile sets `export TMPDIR := /tmp`, which works in a real MSYS2 MinGW
shell. If you build from a sandboxed shell where `/tmp` is not writable, pass a
writable temp explicitly, e.g. `make TMPDIR=C:/msys64/tmp` with
`TMPDIR/TMP/TEMP` set to the same path in the environment. CI runs on Ubuntu
where this is a non-issue.
