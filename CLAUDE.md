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
`tools/tool_eval_lap.c`; the machine-readable summary is the `RESULT ...` line.

Keep evaluation runs at/under **50 s of simulated time** (one lap is ~26.5 s) —
the evaluator already caps at 50 s. Longer runs waste time without adding
signal.

A `TRACE=1 make eval`-style run (set the env var, then run the eval binary
`HIL_Firmware/build/eval_lap`) prints a per-tick trace (waypoint, curvature,
cross-track error, speed, steer, slip) — use it to localise where the car goes
wide.

### Parameter sweeps

Every tuning gain is wrapped in `#ifndef` in its defining header, so it can be
overridden at compile time with `-D` without editing the source. The optimiser
is `tools/tool_smart_sweep_lqr.py`: an adaptive random→converge search that scores
each candidate by its **worst ±3% perturbed neighbour**, so it finds a config in
a clean basin rather than a knife-edge that only laps cleanly at the exact point
(a naive "fastest clean lap" overfits the deterministic evaluator and goes
off-track under tiny drift). `tools/tool_robust_check_lqr.py` audits a config's
robustness. When picking a "fastest" config, only accept one with **0 off-track
ticks** that also survives perturbation — faster laps that clip apex cones, or
that are clean only at one point, are not valid.

The tuned values are the in-source defaults (no `-D` needed for the clean lap).

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
  gains are `#ifndef`-wrapped so `tools/tool_smart_sweep_lqr.py` and `-D` overrides work.

- **Tools live in `tools/`, not `tests/`, and are named `tool_*`**
  (`tool_eval_lap.c`, `tool_perf_sim.c`, `tool_smart_sweep_lqr.py` /
  `tool_robust_check_lqr.py`, plus the CI helpers `tool_compare_eval.py` /
  `tool_eval_common.py`). Unit tests in `tests/` are named `test_*`.

- **The STATE line has 21 fields and yaw rate sits at field 6, before the four
  wheel torques** (fl/fr/rl/rr are fields 7-10). It is very easy to miscount and
  read the yaw-rate column as a wheel torque. The full field order is in the
  visualiser.py header.

- **The steering driver is a model-based LQR law** (`HIL_Firmware/src/lqr_steer.c`),
  not Pure Pursuit or Stanley (older comments/docs may still say either). It
  solves an infinite-horizon LQR on the dynamic-bicycle lateral error dynamics
  (e1=cross-track, e2=heading error) with a speed-scheduled gain, plus a
  curvature feedforward and a cross-track integrator. It tracks the line far
  tighter than the old geometric tracker (mean CTE ~0.09 m). Its tuning knobs
  (Q/R cost weights, `LQR_KI`/`LQR_I_MAX`) live in `lqr_steer.c`, not in
  `parameters_config.h`. Call `lqr_steer_reset()` between independent runs.

- **Torque vectoring acts during braking too.** The left/right bias depends only
  on yaw-rate error, not on the sign of the driver torque, so it is applied to
  regen on corner entry as well as to drive on power. There are no friction
  brakes; the car brakes with motor regen only.

- **The lap-time lever is corner speed (`MAX_LATERAL_ACCEL_MS2`), bounded by the
  racing line.** The LQR tracker holds the line tightly enough to carry nearly
  the full grip budget (~13.7 m/s^2) on the feasibility-aware racing line, for a
  clean ~26.5 s lap. The budget, the line (`PP_GRIP_ACCEL`, `RACING_MARGIN`,
  `PP_MIN_RADIUS_M` in `path_planning.c`), and the LQR cost weights are coupled —
  raise the budget and the line must open the hairpin (`PP_MIN_RADIUS_M`) and add
  apex clearance (`RACING_MARGIN`) or the car saturates the steering and stalls.
  Re-tune them together (use `tools/tool_smart_sweep_lqr.py`) and always confirm 0
  off-track with `make eval`.

- **The TV controller keeps internal PID state** (static integrator and previous
  error). Call `torque_vectoring_reset()` between independent runs or test cases.
  Unit tests must reset between cases or one case leaks state into the next.

- **The ECU boundary is build-enforced.** `torque_vectoring.c` compiles with only
  `-I ECU_Firmware/include -I shared`, so it must not include anything from
  `HIL_Firmware/`. `shared/` is the only place both sides can share code.
