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

`make` may fail to set the temp even with the flag because shell state does not
persist between commands. The reliable fallback is to invoke gcc directly with
the source list from the Makefile and the env set inline on the same line:
`TMPDIR=/c/msys64/tmp TMP=/c/msys64/tmp TEMP=/c/msys64/tmp gcc -std=c11 -O2 -I ... -lm`.

## Key facts and gotchas

These are things that are easy to get wrong and slow to rediscover.

- **All tunable parameters live in `shared/parameters_config.h`.** The car's
  physical constants (mass, geometry, tyre coefficients) are separate, in
  `shared/vehicle_config.h`. Both sides include the config; the highest-leverage
  gains are `#ifndef`-wrapped so `tools/sweep.sh` and `-D` overrides work.

- **Tools live in `tools/`, not `tests/`** (`eval_lap.c`, `perf_sim.c`,
  `sweep.sh`, plus the CI helpers `compare_eval.py` / `eval_common.py`).

- **The STATE line has 21 fields and yaw rate sits at field 6, before the four
  wheel torques** (fl/fr/rl/rr are fields 7-10). It is very easy to miscount and
  read the yaw-rate column as a wheel torque. The full field order is in the
  visualiser.py header.

- **The steering driver is Pure Pursuit, not Stanley** (older comments and docs
  may still say Stanley). It aims a look-ahead point on the line and computes the
  single-arc steer angle.

- **Torque vectoring acts during braking too.** The left/right bias depends only
  on yaw-rate error, not on the sign of the driver torque, so it is applied to
  regen on corner entry as well as to drive on power. There are no friction
  brakes; the car brakes with motor regen only.

- **The lap-time bottleneck is corner speed (`MAX_LATERAL_ACCEL_MS2`), not the
  controller.** The car already tracks its planned speed ~98% of the lap. Raising
  the speed budget is the main lever, but past ~3.7 m/s^2 the car clips apex
  cones even when tracking the line perfectly, because the min-curvature racing
  line hugs those apexes. Going faster than a ~35 s clean lap needs a
  feasibility-aware racing line (path planning), not more speed budget or more TV.

- **`MAX_LATERAL_ACCEL_MS2` and `K_CTE_PP` are coupled.** Raise the speed budget
  and you usually need more cross-track pull to keep the car off the apex cones.
  Tune them together and always confirm 0 off-track with `make eval`.

- **The TV controller keeps internal PID state** (static integrator and previous
  error). Call `torque_vectoring_reset()` between independent runs or test cases.
  Unit tests must reset between cases or one case leaks state into the next.

- **The ECU boundary is build-enforced.** `torque_vectoring.c` compiles with only
  `-I ECU_Firmware/include -I shared`, so it must not include anything from
  `HIL_Firmware/`. `shared/` is the only place both sides can share code.
