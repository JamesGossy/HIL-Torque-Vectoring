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

## Code style

C is formatted with clang-format (`.clang-format` in the repo root; house style
is WebKit-based — Allman braces on functions, attached on control flow, 4-space
indent, 100-col). `make format` reformats every hand-written C file in place;
`make format-check` verifies without editing (non-zero exit if anything is
off-style — suitable for CI). The generated `track_data.h` is excluded (its
layout is owned by `gen_tracks.py`). clang-format does not preserve hand-aligned
columns inside expressions, so don't bother manually aligning operands — it will
be normalised.

## CI

`.github/workflows/ci.yml` builds, runs `make test`, then runs `make eval` and
posts the lap-evaluation table to the GitHub Actions run summary. CI **fails**
if the car does not complete a lap or goes off-track, so a driver regression is
caught automatically.

## Build note (Windows)

**Root cause of the recurring "gcc fails with exit 1 and no error message".**
MinGW gcc writes cc1/as intermediates to a temp dir, and on this machine two
things conspired:
- gcc honours the **Windows-style `TMP`/`TEMP`, not the POSIX `TMPDIR`**, so the
  old `export TMPDIR := /tmp` was silently ignored and temps landed in `%TEMP%`
  under the user profile.
- This repo is under **OneDrive**, and `%TEMP%` is heavily scanned by Defender/
  OneDrive. The scanner races `as` reading cc1's temp `.s`, so the assemble step
  intermittently fails to open its own temp and the build dies with **no
  diagnostic and a bogus exit 1** (the output file sometimes still appears —
  proof the failure is in the temp/exit-status path, not the code).

The Makefile now fixes this: it points `TMPDIR` **and** `TMP`/`TEMP` at a fixed,
local, non-synced dir (`C:\mk_tmp`, auto-created) so cc1/as temps never touch
OneDrive or `%TEMP%`. Just run `make` / `make eval` / `make test` normally.
Override the dir with `make BUILD_TMP=/c/other/tmp` if `C:` is not writable. CI
runs on Ubuntu where none of this applies.

If you must invoke gcc by hand outside make, set the **Windows-style** temp (not
just `TMPDIR`) on the same line, e.g.
`TMP='C:/mk_tmp' TEMP='C:/mk_tmp' gcc -std=c11 -O2 -I ... -lm`.

Note: independent of the above, the sandboxed shell can enter a degraded state
mid-session where gcc cannot spawn its subprocesses at all (even `gcc -S` of a
one-line file fails, while `cc1 --version` and `as --version` run fine). That is
an environment condition, not a repo problem — start a fresh shell/session.

## Key facts and gotchas

These are things that are easy to get wrong and slow to rediscover.

- **The track layout is selectable at runtime via the `TRACK` environment
  variable** (default `fsg2024`). The cone layouts live in `tracks/*.yaml` (the
  source of truth); `tools/gen_tracks.py` turns every YAML into
  `HIL_Firmware/include/track_data.h` at build time (the Makefile runs it before
  compiling — `track_data.h` is generated, do not edit it by hand).
  `HIL_Firmware/src/track_parser.c` includes the generated data and `track_init()`
  picks a layout by name, so every entry point (sim, `make eval`, perf) and the
  visualiser switch with no call-site changes — e.g. `TRACK=fse2024 make eval`.
  Add a track by dropping another `tracks/<name>.yaml` in and rebuilding.
  (`gen_tracks.py --check` fails if the committed header is stale — useful in CI.)

- **The shipped gains are a single shared set tuned to lap BOTH `fsg2024` and
  `fse2024` cleanly** (0 off-track, robust to ±3% jitter), not the fsg2024-only
  optimum. It's a min-max compromise: forcing `fse2024`'s tighter corners clean
  costs `fsg2024` ~0.7 s vs. its solo best (fsg2024 ~27.1 s, fse2024 ~22.1 s).
  Re-tune the shared set with `tools/tool_smart_sweep_lqr_multi.py` (the
  multi-track variant of `tool_smart_sweep_lqr.py`: same adaptive search and
  robust scoring, but it evaluates every candidate on each track in its
  `TRACKS` list and scores by the worst track). After tuning, always confirm 0
  off-track on **every** track, e.g. `TRACK=fse2024 make eval` as well as the
  default. If you only care about one layout, `tool_smart_sweep_lqr.py` still
  optimises that single track.

- **Every number in the project lives in exactly one of two files.**
  `shared/vehicle_config.h` holds everything MEASURABLE on the car or DERIVED from
  it — mass, geometry, tyre/aero coefficients, motor torque limits, top speed,
  gear ratio, the 100 Hz control period, the planner's structural array CAPS, and
  derived quantities (`ACK_NOMINAL` = mid of the Ackermann ratios, `DRIVER_TORQUE_NM`
  = `N_MOTORS × MAX_MOTOR_TORQUE_NM`). `shared/tunables.c` holds every controller
  GAIN as a `g_*` global (there is no `shared/constants_config.h` — it was deleted
  and split between these two files). The classification rule is **"is its value a
  performance/behaviour CHOICE?"** — if tuning it trades lap time, stability or
  cleanliness, it is a gain and lives in `tunables.c` with a `TUNE_*` env override
  and a slot in the sweep's `PARAMS`. That deliberately includes the steering caps
  (`g_MAX_STEER_RAD`/`g_MAX_STEER_RATE_RADS` — the sim enforces no mechanical lock,
  so they are driver choices), the braking-effort cap (`g_MAX_BRAKE_DECEL_MS2`, a
  choice below the regen limit), the longitudinal P/I gains, the speed-planner scan
  DEPTH (`g_SPEED_PLAN_STEPS`/`g_NEAREST_SEARCH_*`, ints clamped to the structural
  caps), the cone safety net, the racing-line radius-floor opening
  (`g_PP_RADIUS_FACTOR`), and the TV PID/FF fractions (incl. `g_TV_I_MAX_FRAC`, the
  integral cap as a fraction of motor torque). Only **two** gains are FIXED (not
  swept) because they are NOT performance choices: `g_TV_YAW_DEADBAND` (a
  sensor-noise floor) and `g_TV_K_US` (an empirical understeer term whose linear
  derivation collapses to ~0 for this near-neutral car). The headline swept knobs
  are `g_GRIP_USE`, `g_K_STANLEY`, `g_K_DAMP`, `g_RACING_MARGIN`, `g_KP_YAW`; the
  full set (~26) is in `tunables.c` and the multi-track sweep's `PARAMS`. **Much
  else is DERIVED** from `vehicle_config.h`: the single grip model `peak_lat(v)` in
  `shared/grip_model.h` feeds the speed planner, friction-circle budget and
  throttle cut; the steering feedforward/understeer come from the Pacejka
  stiffness; the drag feedforward from the aero constants; `ACK_NOMINAL` is the
  midpoint of the Ackermann ratios; the racing-line radius floor from the steering
  geometry. (This replaced an earlier 16-tunable set whose steering was a
  model-based LQR; see the steering note below for the pace trade-off.)

- **Tools live in `tools/`, not `tests/`, and are named `tool_*`**
  (`tool_eval_lap.c`, `tool_perf_sim.c`, `tool_smart_sweep_lqr.py` /
  `tool_robust_check_lqr.py`, plus the CI helpers `tool_compare_eval.py` /
  `tool_eval_common.py`). Unit tests in `tests/` are named `test_*`.

- **The STATE line has 21 fields and yaw rate sits at field 6, before the four
  wheel torques** (fl/fr/rl/rr are fields 7-10). It is very easy to miscount and
  read the yaw-rate column as a wheel torque. The full field order is in the
  visualiser.py header.

- **The steering driver is a Stanley law plus a physics-derived curvature
  feedforward**, inline in `HIL_Firmware/src/motion_control.c` (`steer_command`).
  It replaced an earlier model-based LQR (the deleted `lqr_steer.c`); older
  comments/docs may still say LQR, Pure Pursuit or Stanley — it is now Stanley.
  The feedforward `δ = (L·κ + Kus·v²·κ)/ACK_NOMINAL` uses the understeer gradient
  `Kus` derived from the Pacejka cornering stiffness (no tuning); the feedback is
  `−e2 + atan2(K_STANLEY·(−e1), v)` on heading error e2 and cross-track e1. The
  one steering tunable is `g_K_STANLEY`. It keeps NO integrator, so there is no
  steering state to reset (mean CTE ~0.16 m, a touch looser than the old LQR's
  ~0.09 m but 0 off-track and ~same lap time). `motion_control_reset()` clears
  only the progress index and throttle integrator.

- **Torque vectoring acts during braking too.** The left/right bias depends only
  on yaw-rate error, not on the sign of the driver torque, so it is applied to
  regen on corner entry as well as to drive on power. There are no friction
  brakes; the car brakes with motor regen only.

- **The lap-time lever is corner speed, set by `g_GRIP_USE`** (the fraction of
  the physically-derived peak grip the car drives at), bounded by the racing line.
  The grip budget feeds the speed planner, the friction-circle braking budget and
  the throttle traction cut through the single `peak_lat(v)` model, so there is
  one grip number, not four. The racing line is shaped for the FULL physical peak
  (its grip and radius floor are derived, not tuned), so `g_RACING_MARGIN` is the
  line's only knob. Raising `g_GRIP_USE` toward 1 carries more speed but leaves
  the Stanley tracker less margin; re-tune with
  `tools/tool_smart_sweep_lqr_multi.py` (shared two-track set) or
  `tool_smart_sweep_lqr.py` (one track) and always confirm 0 off-track with
  `make eval` on every track. (The sweep tools now inject the five swept tunables
  via `TUNE_*` env vars on a binary built once — no recompile per candidate.)

- **The TV controller keeps internal PID state** (static integrator and previous
  error). Call `torque_vectoring_reset()` between independent runs or test cases.
  Unit tests must reset between cases or one case leaks state into the next.

- **The ECU boundary is build-enforced.** `torque_vectoring.c` compiles with only
  `-I ECU_Firmware/include -I shared`, so it must not include anything from
  `HIL_Firmware/`. `shared/` is the only place both sides can share code.
