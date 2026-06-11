# Closed-Loop Autonomous Driving with EKF-SLAM

## Context

Today the sim is not autonomous: the controller reads the car's **ground-truth**
pose (`state->x/y/heading`) directly, and steers along a racing line that
`path_plan()` computes **once offline** from the known cone layout. That is a
perfect-information driver, not autonomy.

The goal is full closed-loop autonomy: the car drives **only** on its own
estimates. A simulated cone-detection sensor produces noisy range/bearing
observations; an EKF-SLAM backend fuses those with wheel-speed/yaw-rate odometry
to estimate both the car pose and a cone map; the car plans and drives from that
estimate. Ground truth is used **only** for synthesising the sensor input and for
scoring. Because the sim is 2D there is no camera to model, so perception is
abstracted to a geometric cone detector (FoV + range + Gaussian noise) feeding a
real SLAM algorithm.

Decisions made with the user:
1. **Full closed-loop autonomy** (control on SLAM output only).
2. **EKF-SLAM** built from scratch in C (no LA library exists).
3. **Two-phase planning**: lap 1 follows a reactive local centreline while
   mapping; after loop closure, run `path_plan()` on the completed map and race
   the optimised line on later laps.
4. **Eval reports both** SLAM pose/heading RMSE + map RMSE **and** the existing
   lap metrics (off-track, lap time), with the car driving autonomously.
5. **The whole autonomy stack runs ON THE ECU.** The ECU owns SLAM + planner +
   motion control + torque vectoring. The HIL side becomes a pure plant + sensor
   synthesiser. `slam.c`, `online_planner.c`, and `motion_control.c` **move into
   `ECU_Firmware/`**; `cone_sensor.c` stays HIL-side (it synthesises sensor
   input from ground truth). The ECU IS the driver now, so `driver_torque` leaves
   `SensorData`.

## The boundary change (this is the core of the revised design)

Today the HIL side is the "driver": it runs `motion_control` on ground-truth
pose + the offline racing line, and the ECU only does torque vectoring. Once the
car is autonomous, the **driver is the ECU**. The HIL/ECU contract in
shared/tv_interface.h is rewritten:

- **`SensorData` (HIL -> ECU)** gains exteroception and loses `driver_torque`:
  `yaw_rate`, `velocity`, `steering_angle`, `wheel_speed[4]`, plus a per-tick
  `ConeScan` (the noisy cone observations). Move `ConeObservation`/`ConeScan`
  into `tv_interface.h` so both sides share the contract.
- **ECU output** becomes steering + torque, not just torque: extend the ECU API
  to return a `DriveCommand { float steering_rad; WheelTorques torques; }`. The
  HIL applies `steering_rad` to the plant (writes `state->steering`) and the
  torques to the motors.
- The ECU is the only place SLAM/planner/control run, and it sees **only**
  `SensorData` — never `VehicleState` or the ground-truth `Track`. This is the
  same build-enforced boundary as today, now carrying the full autonomy stack.

**Severing the HIL dependency from `motion_control`/`path_planning`** (the real
work of the move): both currently depend on `VehicleState` and the HIL `Track`
(which includes the generated `track_data.h`). ECU-side they must not. So:

- `motion_control` stops taking `VehicleState`/`Track`. It takes the `CtrlPose`
  (from SLAM) + an **ECU-internal map type** (the SLAM-built cone map + planned
  line), and returns steering + torque demand instead of writing
  `state->steering`. The `CtrlPose` seam from the prior design already does the
  pose half of this.
- The planner's `Track`-shaped data becomes an **ECU-internal `EcuMap`** type
  (cone arrays + waypoint arrays + progress index) declared in
  `ECU_Firmware/include`, not the HIL `track_parser.h` `Track`. `path_plan`'s
  geometry is pure (cones in -> waypoints out), so it ports cleanly once it
  refers to `EcuMap` instead of `Track`. The HIL ground-truth `Track` stays
  HIL-side and is used only for the cone sensor and scoring.

## Guiding constraints

- Match the repo's minimal C style: fixed-cap arrays (no hot-path `malloc`),
  static module-local scratch, `g_*` tunables with `TUNE_*` env overrides
  (shared/tunables.c), `-lm` only.
- **The existing ground-truth code path must keep working unchanged**, gated by a
  `g_AUTONOMY` flag (default 0). Every milestone stays bisectable with `make eval`.
- HIL/ECU boundary untouched: cone_sensor and slam are HIL-only modules;
  ECU_Firmware/src/torque_vectoring.c still sees only `SensorData`.

## Coordinate frames (lock down first — top divergence source)

- World frame: x=east, y=north, heading CCW+ (as `VehicleState` already uses).
- SLAM pose `[px,py,theta]` is in the **same** world frame, seeded at the known
  start pose (car always starts at `track.points[0]` facing segment 0). So pose
  RMSE compares to ground truth with no alignment transform. Legitimate: an FS
  car knows its start box.
- Observations are **body-frame** range/bearing, bearing from heading, CCW+
  (left positive — must match `steer_command`'s `e1` left-positive convention).

## New / moved files

HIL-side (plant + sensor):
- HIL_Firmware/src/cone_sensor.c + .h — simulated detector (synthesises from ground truth)

ECU-side (autonomy stack — must compile with only `-I ECU_Firmware/include -I shared`):
- ECU_Firmware/src/slam.c + include/slam.h — EKF-SLAM core (new)
- ECU_Firmware/src/online_planner.c + include/online_planner.h — two-phase planner (new)
- ECU_Firmware/src/motion_control.c + include/motion_control.h — **moved** from HIL_Firmware/, dependencies severed
- ECU_Firmware/src/path_planning.c + include/path_planning.h — **moved** (used by both the offline-equivalent and the phase-2 planner)
- ECU_Firmware/include/ecu_map.h — ECU-internal `EcuMap` type (replaces the HIL `Track` for control/planning) [DONE]
- ECU_Firmware/src/autopilot.c + include/autopilot.h — top-level ECU entry: `autopilot_update(const SensorData*, DriveCommand*)` ties SLAM -> planner -> motion_control -> torque_vectoring

Shared / contract:
- shared/tv_interface.h — extended `SensorData` (+ `ConeScan`, − `driver_torque`), new `DriveCommand`
- shared/linalg.h — header-only tiny matrix ops (ECU includes it)

Tests:
- tests/test_slam.c

## 1. Cone-detection sensor

`cone_sensor_scan(track, gt_x, gt_y, gt_heading, *scan)` synthesises noisy
observations from ground truth (HIL-only; not ECU). For each L/R cone (O(n) loop,
fine at <=150/side): compute range/bearing relative to the true pose, reject if
`range > g_SENSOR_RANGE_M` or `|bearing| > g_SENSOR_FOV_RAD` (half-angle), add
Gaussian noise (Box-Muller from a module-static seed so eval is deterministic;
`cone_sensor_reset()` reseeds). Color = source array (perfect classification v1).
Cap at `MAX_OBS_PER_TICK`, keep nearest.

```c
typedef struct { float range, bearing; int color; } ConeObservation;
typedef struct { ConeObservation obs[MAX_OBS_PER_TICK]; int count; } ConeScan;
```

New tunables (the sigmas double as the EKF measurement noise `R`, keeping filter
and sensor consistent by construction): `g_SENSOR_RANGE_M` (~12), `g_SENSOR_FOV_RAD`
(~1.2), `g_SENSOR_RANGE_SIGMA_M` (~0.10), `g_SENSOR_BEARING_SIGMA_RAD` (~0.02).

**Injection point** (in HIL_Firmware/src/main.c and
tools/tool_eval_lap.c): after `vehicle_model_update`, before
`track_update` — order is control -> physics -> sense -> SLAM -> progress.

## 2. Linear algebra — shared/linalg.h

Header-only `static inline`, included by `slam.c` only (no Makefile change).
**Do not build a generic NxN library.** Exploit EKF-SLAM sparsity. The only dense
inverse ever needed is **2x2** (innovation covariance `S` for one range/bearing
obs) — closed-form `inv2x2` (returns 0 on singular). Plus small fixed-shape
`mat_mul`, `mat_mul_T` (for `P*H^T`), `mat_add`, `mat_copy`, `vec_sub2`,
`wrap_angle`.

## 3. EKF-SLAM — ECU_Firmware/src/slam.c

Fixed-cap, full dense covariance (no sparse storage — adds bookkeeping for no
benefit at this scale). `SLAM_MAX_LANDMARKS = 2*ECU_MAX_CONES` (300); `SLAM_MAX_DIM =
3 + 2*300 = 603`; `P` is ~1.45 MB static — confirm acceptable, else fall back to
`ECU_MAX_CONES` and document.

```c
typedef struct { int color, seen_count, slot; } Landmark;
typedef struct {
    float mu[SLAM_MAX_DIM];              // [px,py,theta, lx0,ly0, ...]
    float P[SLAM_MAX_DIM*SLAM_MAX_DIM];
    int n_land, dim;                     // dim = 3 + 2*n_land
    Landmark land[SLAM_MAX_LANDMARKS];
    int start_land_ids[8], n_start_land, loop_closed;
} SlamState;
```

API: `slam_init(s, start_x, start_y, start_heading)`,
`slam_predict(s, sensors, dt)`, `slam_update(s, scan)`, `slam_loop_closed(s)`,
`slam_get_pose(s, *x,*y,*h)`, `slam_export_cones(s, *out_map)`.

**Predict (odometry, NOT ground truth):** `v = sensors.velocity`,
`w = sensors.yaw_rate` (autonomy contract). Unicycle: `px+=v*cos(theta)*dt;
py+=v*sin(theta)*dt; theta=wrap(theta+w*dt)`. Jacobian `F` (3x3) over pose. Only
the pose block and pose-landmark cross blocks update (`P_pp=F P_pp F^T + Q`,
`P_pl=F P_pl`); landmark-landmark block untouched (sparsity win). `Q` additive,
tunable `g_SLAM_Q_POS`, `g_SLAM_Q_THETA`. `sensors.velocity` understates speed
under slip — accepted; measurement updates correct it.

**Update:** for landmark `(lx,ly)`: `dx=lx-px, dy=ly-py, q=dx^2+dy^2, r=sqrt(q)`,
`z_hat=[r; wrap(atan2(dy,dx)-theta)]`. `H` is 2x5 over `[px,py,theta,lx,ly]`
(nonzero only there). Compute `H*P`/`P*H^T`/`S=H P H^T + R` using only the 5
active columns (hand block, not full 2xdim). `K=P H^T S^-1`; `mu+=K y` (wrap
`mu[2]` and innovation angle); `P -= K(H P)` rank-2 downdate.

**Data association** (nearest + Mahalanobis gating): color-gate first (cheap,
prevents blue/yellow cross-association), then min `d2 = y^T S^-1 y` over
same-color landmarks. Associate if `d2 < g_SLAM_GATE_CHI2` (~5.99). Initialise a
NEW landmark only if `d2 > g_SLAM_NEW_CHI2` **and** world position is
> `g_SLAM_NEW_MIN_DIST_M` from all same-color landmarks (dual guard defeats
double-counting). Process nearest observations first.

**Landmark init:** inverse measurement `lx=px+r*cos(theta+b), ly=py+r*sin(theta+b)`;
append at `slot=dim`, `dim+=2`. Seed `P_LL` large-diagonal (e.g. `diag(2,2)`),
zero cross terms (EKF tightens it) — pragmatic v1. Record color, bump
`seen_count`, track first ~8 ids for loop closure.

**Loop closure:** fire once when the car has left the start (cumulative travel
> 15 m, mirroring eval's `moved_away`) **and** returned within
`g_SLAM_LOOP_RADIUS_M` while re-observing a `start_land_ids` landmark.

## 4. Control on the ECU — moved motion_control.c

`motion_control`'s steering math (`steer_command`, `find_nearest_segment`,
`plan_target_speed`) is pure geometry and ports unchanged. What changes is its
**inputs and outputs**, to sever the HIL dependency:

```c
typedef struct { float x, y, heading, vx, yaw_rate; } CtrlPose;   // from SLAM

// ECU-side: pose + ECU map in, steering + torque demand out. No VehicleState, no HIL Track.
float motion_control_update(const CtrlPose *pose, const EcuMap *map,
                            float *out_steering_rad, float *out_target_speed);
void  motion_control_reset(void);
```

- Every old `state->x/y/heading` read becomes `pose->...` (SLAM estimate).
- Steering is **returned** (`*out_steering_rad`) instead of written to
  `state->steering`; the HIL applies it to the plant.
- The `Track*` argument becomes `EcuMap*` (same waypoint/progress fields, ECU
  type). `find_nearest_segment` / curvature / `plan_target_speed` operate on
  `map->points[]` exactly as before.

**Top-level ECU entry** `autopilot_update(const SensorData *s, DriveCommand *cmd)`
(in `autopilot.c`) is the single ECU tick: `slam_predict(s)` ->
`slam_update(s->scan)` -> `online_planner_step` (phase 1 or phase 2) ->
`motion_control_update(pose, map, &steer, &tgt)` -> `torque_vectoring_update(...)`
using the planner's torque demand. It fills `cmd->steering_rad` and
`cmd->torques`. Torque vectoring's existing PID is reused as-is (it already takes
only `SensorData` + a torque demand + `kp_yaw`).

**HIL wiring** (HIL_Firmware/src/main.c /
tools/tool_eval_lap.c): per tick — `cone_sensor_scan` into
`sensors.scan`; pack proprioception into `sensors`; `autopilot_update(&sensors,
&cmd)`; apply `cmd.steering_rad` to `state->steering` and `cmd.torques` to
`vehicle_model_update`. Ground-truth `state`/`track` are used only by the cone
sensor (input) and scoring. A `g_AUTONOMY=0` path keeps the legacy
ground-truth driver for the step-0/step-1 regression baseline (see rollout).

## 5. Two-phase planner — ECU_Firmware/src/online_planner.c

`extract_gates` in `path_planning.c` (now ECU-side) assumes **cones already in
track order** — load-bearing for both phases. Both planner and `path_plan` now
operate on `EcuMap`, not the HIL `Track`.

**Phase 1 (lap 1, reactive centreline)** while `!slam_loop_closed`, every N
ticks: `slam_export_cones` confident landmarks (`seen_count >=
g_SLAM_MIN_SIGHTINGS`) into the `EcuMap` cone arrays; build a short local
centreline ahead of the estimated pose (mapped L/R cones within ~15 m forward,
pair nearest L<->R, midpoints, ordered by forward distance) into `map->points`.
Cap speed at `g_PHASE1_SPEED_CAP_MS` (~6 m/s) so association holds and mapping is
clean. Reuse the gate-pairing/midpoint logic from `path_planning.c`.

**Loop-closure handoff:** export the **full** SLAM map; chain-sort each color
array by nearest-neighbour from the start cone (so `extract_gates`' track-order
assumption holds); call `path_plan(map)` once (exactly as `track_init` did
offline, but on the estimated `EcuMap`); call `motion_control_reset()` to clear
the stale progress index; set `phase2_active`. Laps 2+ race the optimised line.

## 6. Build wiring — Makefile

The ECU autonomy sources (`slam.c`, `online_planner.c`, `motion_control.c`,
`path_planning.c`, `autopilot.c`, plus the existing `torque_vectoring.c`) compile
under the **ECU flags** (`-I ECU_Firmware/include -I shared`, no `HIL_Firmware/`)
— so the boundary stays build-enforced and a stray HIL include fails to compile.
`cone_sensor.c` compiles HIL-side. Add the new ECU sources to the ECU object set
used by `hil_sim`, `eval`, integration and perf targets; `cone_sensor.c` to the
HIL set. The `%.o` pattern rules need a rule for `ECU_Firmware/src/%.c` if not
already present. Add `g_SENSOR_*`, `g_SLAM_*`, `g_AUTONOMY`, `g_PHASE1_*` to
shared/tunables.h / shared/tunables.c (already shared, so ECU-visible) via the
`getenvf`/`getenvi_clamped` pattern. Add `TEST_SLAM`/`SLAM_SRCS` to the `test:`
recipe (compiled with ECU flags to prove the boundary). New files must pass
`make format-check`.

## 7. Tests — tests/test_slam.c

Same harness as siblings (ASSERT/ASSERT_NEAR, `main` returns 0/1). Cover:
linalg (`inv2x2` vs `A*A^-1=I`, singular returns 0; `mat_mul`; `wrap_angle` at
±pi); predict grows pose covariance trace; consistent updates **shrink** the
trace and converge a landmark to truth within sigma; association does not merge
distinct-color cones and does not double-count a re-seen same-color cone; loop
closure flips exactly once; full scripted traversal yields map RMSE < ~0.3 m.

**Eval extension** (tools/tool_eval_lap.c): with `g_AUTONOMY`
on, the loop calls `autopilot_update` (ECU owns SLAM + planner + control + TV).
Accumulate pose RMSE (vs ground-truth `state`) per tick; compute map RMSE at the
end (each landmark vs nearest true cone of same color). CTE/off_track/lap metrics
stay ground-truth based. Extend `RESULT`: `... pose_rmse_pos=.. pose_rmse_head=..
map_rmse=.. laps=.. lap_s=..`. CI gate per requirement 4: fail if `offtrack>0` or
`laps<1` (keep exit-code change behind `CI=1` to avoid breaking sweep tooling).

## 8. Phased rollout (each independently `make eval`-verifiable)

- **Step 0 — move + reshape, no behaviour change.** Move `motion_control.c` /
  `path_planning.c` to `ECU_Firmware/`; introduce `EcuMap`, `CtrlPose`,
  `DriveCommand`; sever HIL includes; add `linalg.h`. Keep `g_AUTONOMY=0` legacy
  path where the HIL fills `EcuMap` from the ground-truth track and `CtrlPose`
  from ground-truth pose, calling the moved control. `make test && make eval`
  must produce **identical** numbers to today (proves the move is behaviour-
  preserving and the boundary still compiles). Commit as baseline.
- **Step 1 — cone sensor + SLAM running, control still on truth** (`g_AUTONOMY=0`).
  Eval prints RMSE diagnostics; lap metrics unchanged; RMSE finite and shrinking.
  Proves SLAM converges before it steers.
- **Step 2 — localisation on** (`g_AUTONOMY=1`, SLAM pose, but `EcuMap` still
  seeded from the true line). Car laps clean on estimated pose. Tune `g_SLAM_Q_*`
  / gates.
- **Step 3 — online map + reactive centreline** (full closed loop, phase 1, speed
  cap; `autopilot_update` fully owns the tick). Hardest milestone (association at
  speed, map quality). Tune sensor range/FoV and phase-1 cap.
- **Step 4 — loop closure + racing line** (phase 2). Lap 2 faster than lap 1,
  still 0 off-track, map RMSE within ceiling. Final acceptance = autonomous eval
  green on both tracks.

## 9. Risks & edge cases

- **Association at speed**: phase-1 cap; nearest-first; color-gate before
  Mahalanobis. If flickering, tighten `g_SLAM_GATE_CHI2`, raise
  `g_SLAM_NEW_MIN_DIST_M`.
- **EKF divergence**: guard `inv2x2` singularity (skip update, no NaN); wrap every
  angle touch (the classic cause); seed landmark covariance generously.
- **Double-counting**: dual Mahalanobis + Euclidean new-landmark guard;
  `seen_count` confidence gate before a landmark enters the planner map.
- **Lap-1 speed vs 50 s budget**: tune `g_PHASE1_SPEED_CAP_MS` so lap 1 fits with
  margin.
- **MAX_LANDMARKS / 1.45 MB `P`**: assert `n_land` never exceeds cap; confirm the
  static array size is acceptable.
- **Odometry drift under slip**: accepted; updates correct it; slightly inflate
  `Q` on theta for yaw-rate lag.
- **`path_plan` track-order assumption**: chain-sort the SLAM map by color before
  the phase-2 `path_plan` call.
- **Loop-closure false trigger**: require both left-start and returned-and-re-seen.

## Verification

- `make test` (includes new `test_slam`) — all pass.
- `make eval` after step 0 — byte-identical RESULT to pre-change baseline.
- `make eval` with autonomy at step 4 — `offtrack=0`, `laps>=1`, lap 2 < lap 1,
  finite `pose_rmse_*`/`map_rmse` within sanity ceilings; confirm on **both**
  tracks (`TRACK=fse2024 make eval` as well as default), per CLAUDE.md.
- `make format-check` clean on all new files.
- Visualiser spot-check: overlay SLAM-estimated cones/pose against ground truth
  to eyeball map quality (optional, diagnostic only).

---

## Progress checklist

### Step 0 — move + reshape (byte-identical eval target)
- [x] ECU_Firmware/include/ecu_map.h — EcuMap type
- [x] ECU_Firmware/include/path_planning.h — path_plan(EcuMap*)
- [x] ECU_Firmware/src/path_planning.c — moved, Track->EcuMap, MAX_*->ECU_MAX_*
- [x] ECU_Firmware/include/motion_control.h — CtrlPose + new signature
- [x] ECU_Firmware/src/motion_control.c — moved, drives on CtrlPose+EcuMap, returns steering
- [x] HIL_Firmware/src/main.c — track_to_ecu_map adapter + per-tick CtrlPose, apply returned steering
- [x] tools/tool_eval_lap.c — same adapter + per-tick CtrlPose
- [x] Delete old HIL_Firmware/src/motion_control.c + include/motion_control.h
- [x] Delete old HIL_Firmware/src/path_planning.c + include/path_planning.h
- [x] Update tests: test_motion_control.c, test_steer.c, test_path_planning.c, test_integration.c to new ECU includes/signature
- [x] Update tool_perf_sim.c to new ECU includes/signature
- [x] Update Makefile: move MC/PP sources to ECU paths, add ECU_Firmware/src/%.o rule under HIL_FLAGS, fix test src lists
- [x] make test passes (all suites green; integration 1530/1530)
- [x] make eval byte-identical RESULT (fsg2024 lap_s=26.73, fse2024 lap_s=21.52 vs HEAD baseline — exact match)
- [x] make format-check: clang-format installed (PyPI clang-format 22.1.5 at
      %LOCALAPPDATA%/Programs/Python/Python312/Scripts/clang-format.exe). v22 reports
      line-wrap diffs on ALL files including untouched committed ones (vehicle_model.c,
      torque_vectoring.c, tunables.c) -> it is NEWER than the version that formatted the
      repo. New files are consistent with the committed tree's style; do NOT mass-reformat
      against v22 (would churn the whole tree and diverge from CI's clang-format). Run
      `make format CLANG_FORMAT=<that path>` only if CI's clang-format matches v22.
- [ ] commit baseline

### Step 1 — cone sensor + SLAM logging (control still on truth) DONE
- [x] shared/linalg.h (mat_mul, mat_mul_T, inv2x2, wrap_angle)
- [x] shared/tv_interface.h: ConeObservation/ConeScan in SensorData, DriveCommand added (driver_torque kept for legacy path, dropped at Step 3)
- [x] HIL_Firmware/src/cone_sensor.c + .h (FoV+range+Gaussian noise, deterministic RNG)
- [x] ECU_Firmware/src/slam.c + include/slam.h (predict + update + assoc + loop-closure + export)
- [x] tunables: g_SENSOR_*, g_SLAM_*, g_AUTONOMY, g_PHASE1_*
- [x] eval prints pose/map RMSE; lap metrics byte-identical
- [x] tests/test_slam.c (16/16) wired into make test under ECU flags
- [x] KEY FIX: proper inverse-observation landmark init (pose-landmark cross-covariance).
      Diagonal seed left the pose uncorrectable -> 19m drift. With cross-cov:
      fsg2024 pose_rmse=0.55m map_rmse=0.19m; fse2024 pose_rmse=0.30m map_rmse=0.083m.
      SLAM_TRACE=1 env on eval binary prints per-50-tick pose-error trace.

### Step 2 — localisation on (IN PROGRESS, partial)
- [x] g_AUTONOMY=1 path in eval: CtrlPose from slam_get_pose + sensed v/yaw; EcuMap
      still the true line; scoring stays ground-truth.
- [x] EKF math fully verified (the real debugging win):
      observation Jacobian matches numeric to 4dp; inverse-obs init Jacobians (Gp,Gz)
      match numeric; predict cov matches brute-force F P F^T + Q exactly; Joseph-form
      covariance update (pos-definite); data-association ambiguity rejection via new
      g_SLAM_AMBIG_RATIO (accept match only if runner-up d2 > ratio*best d2).
- [x] fse2024 closed-loop: CLEAN (offtrack=0, laps=2, pose ~0.3m).
- [~] fsg2024 closed-loop: best Q_POS=0.005 -> offtrack=3, pose 0.46m, but KNIFE-EDGE
      (pose swings 0.46->2.2->3.5m across Q 0.004/0.005/0.006). Not robust. Slowing via
      grip_use does NOT robustly fix -> closed-loop feedback coupling, not pure speed
      or a math bug.
- [ ] NEXT SESSION: tame fsg2024 Q-sensitivity. Candidates: (a) feed control a
      lagged/smoothed SLAM pose; (b) gate trust on covariance, hold last-good otherwise;
      (c) proper Step-3 phase-1 hard speed cap + reactive centreline (the plan remedy;
      may differ from the grip_use proxy).
- Diagnostics: SLAM_TRACE=1 on eval_lap; tests/test_slam_motion.c fusion probe (feeds
  TRUE odometry, so its dead-reckoning baseline is artificially perfect -> use relatively).

### Step 3 — online map + reactive centreline (IN PROGRESS, major progress)
- [x] ECU_Firmware/src/online_planner.c + .h: phase-1 reactive centreline from SLAM
      gate midpoints; phase-2 persistent racing_line (planned once at loop closure,
      re-served each tick); chain_sort for track-order; online_planner_lookahead.
- [x] ECU_Firmware/src/autopilot.c + .h: autopilot_update ties SLAM -> planner ->
      control -> TV. Phase 1 uses PURE-PURSUIT (lookahead gate midpoint) not Stanley
      (Stanley keyed off one jittery local segment -> full-lock spin; pure pursuit is
      smooth). Phase 2 uses the Stanley motion_control on the optimised line.
- [x] eval wired: g_AUTONOMY=1 calls autopilot_update; sense->autopilot->apply->physics.
- [x] BUGS FIXED: (1) path_plan div-by-zero guard for <3 gates; (2) phase-2 was
      re-exporting the SLAM map every tick and wiping the planned line + crashing
      (segfault) -> now persisted in planner.racing_line; (3) motion_control_reset at
      phase transition.
- [x] SLAM excellent end-to-end at the 6 m/s phase-1 cap: pose 0.2-0.6m both tracks.
- [x] fse2024: completes an autonomous lap on its SELF-BUILT map (laps=1, lap_s=33.5).
- [~] NOT YET CLEAN: phase-1 pure-pursuit follows the rough midpoint line but wanders
      enough to clip cones (fse offtrack=61; fsg offtrack=15 and drives off a sharp
      section ~t=33 before closing the loop). worst_cte still high.
- [~] PHASE-1 TUNING ATTEMPTED (lookahead length, corner slowdown, speed cap): each
      moves the numbers around (fsg ~10 offtrack, fse ~60-70) but none reaches 0.
      ROOT CAUSE is geometric, not tuning: the centreline pairs each left cone with
      its NEAREST right cone and takes midpoints; through tight corners that nearest
      pairing skews the midpoints (cuts across the corner), so the reference line the
      car follows is itself wrong in exactly the sharp sections where it runs off.
- [x] Tried 3 centreline constructions: (1) nearest-pair raw midpoints; (2) chain-order
      both walls + index-pair; (3) chain-order + nearest-pair + 3pt moving-average smooth.
      Current = (3). Per-side chain_order on the ahead-window + nearest-pair + smoothing.
- [~] BEST SO FAR (construction 3): fse2024 reaches offtrack=0 (!) but doesn't complete
      the lap (pose drifts ~1.9m late, loses the line). fsg2024 still ~52 offtrack
      (its sharp corners scramble the ahead-only chain-walk order). Tracks want
      different behaviour -> not yet a single robust config.
- [ ] NEXT (deliberate controls pass, not rapid tuning): the corridor follower needs
      a construction robust to (a) asymmetric visible cone counts per wall and (b) sharp
      corners scrambling a greedy nearest-next chain. Candidates not yet tried: proper
      Delaunay/edge-midpoint centreline (standard FS); or constrain chain_order by the
      forward direction (reject backward steps); or fuse the confident persistent map
      (not just the ahead window) so ordering is stable. fse2024 offtrack=0 shows the
      pure-pursuit + smoothed-centreline approach CAN work; it needs a more robust
      centreline build to generalise + complete the lap.
- Milestone reached this session: full autonomous stack runs end-to-end on the ECU,
  SLAM verified + accurate (0.2-0.6m), phase-2 racing line wired, fse2024 maps its
  whole track and (separately) achieves a 0-cone-contact partial run.

### Step 3 (original plan text) — online map + reactive centreline (phase 1)
- [ ] ECU_Firmware/src/online_planner.c + .h
- [ ] ECU_Firmware/src/autopilot.c + .h (autopilot_update)
- [ ] full closed loop, phase-1 speed cap; clean lap

### Step 4 — loop closure + racing line (phase 2)
- [ ] loop-closure -> path_plan handoff; lap2 < lap1; clean on both tracks

### Tests + finish
- [ ] tests/test_slam.c
- [ ] eval RESULT line extended (pose_rmse_*, map_rmse)
- [ ] make format-check on all new files
