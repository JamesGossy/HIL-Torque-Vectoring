#ifndef CONSTANTS_CONFIG_H
#define CONSTANTS_CONFIG_H

/*
 * shared/constants_config.h
 *
 * Fixed configuration constants: actuator limits, control-loop timing, fixed
 * controller coefficients, and safety thresholds that are NOT swept. These are
 * compile-time #defines used across the driver and the ECU.
 *
 * The gains the parameter sweep optimises (corner-speed budget, GG budget,
 * downforce fraction, LQR cost weights, racing-line shape, the TV yaw gains)
 * are NOT here - they are runtime tunables (g_* globals) in shared/tunables.c,
 * overridable per run via TUNE_* env vars without a recompile. This header holds
 * only what stays fixed. The car's physical constants (mass, geometry, tyre
 * coefficients) live separately again, in shared/vehicle_config.h.
 */


/* ============================================================ */
/* DRIVER: steering (model-based LQR)                           */
/* ============================================================ */

/* Steering is a model-based LQR law on the dynamic-bicycle lateral error
 * dynamics. Its tuning knobs (the Q/R cost weights and the cross-track
 * integrator) live with the controller in HIL_Firmware/src/lqr_steer.c, since
 * they are meaningless without the model alongside them. Only the actuator-level
 * limits that bound any steering law live here. */

/* Steering reference limit. The vehicle model scales this by the Ackermann
 * ratios (~0.20-0.26) to get the road-wheel angle, so the reference is much
 * larger than the wheel angle. 1.7 gives R_min ~3.8 m, enough for the hairpins. */
#define MAX_STEER_RAD 1.7f

/* Throttle fades out as the steer command rises above this fraction of the
 * limit. Near full lock the front tyres are saturated turning, so throttle
 * only pushes the car wide. */
#define STEER_SAT_FRAC 0.7f

/* Max rate the steering command can move, rad/s. A real actuator cannot snap
 * the wheel instantly. Per-tick step is this * CONTROL_DT_S. */
#ifndef MAX_STEER_RATE_RADS
#define MAX_STEER_RATE_RADS 8.0f
#endif

/* Window of racing-line segments searched around the controller's own progress
 * index when projecting the car onto the line. */
#define NEAREST_SEARCH_BACK 3
#define NEAREST_SEARCH_FWD  30


/* ============================================================ */
/* DRIVER: speed planner                                        */
/* ============================================================ */

/* Cruise speed on the straights, m/s. */
#ifndef TARGET_SPEED_MS
#define TARGET_SPEED_MS 30.0f
#endif

/* Corner-speed budget (g_MAX_LATERAL_ACCEL_MS2, a runtime tunable in
 * shared/tunables.c): v_corner = sqrt(a_lat / kappa). This is the dominant
 * lap-time lever on this corner-heavy track. It is coupled with the racing line
 * (g_PP_GRIP_ACCEL, shaped for the same budget) and the hairpin radius floor
 * (g_PP_MIN_RADIUS_M, which must open as the budget rises, or the car saturates
 * the steering and stalls at the hairpin). Re-tune them together with
 * tools/tool_smart_sweep_lqr_multi.py and confirm 0 off-track with `make eval`. */

#define MAX_BRAKE_DECEL_MS2 5.6f /* max straight-line braking decel, m/s^2 */

/* Friction-circle (GG) budget, m/s^2. The tyre shares ONE grip budget between
 * cornering and braking: a_lat^2 + a_lon^2 <= GG_ACCEL_MS2^2. The planner uses
 * this so that braking INTO a corner is limited by the lateral load already in
 * use - as the car approaches a tight apex, a_lat = v^2*kappa rises and eats the
 * circle, leaving less for braking, which forces an earlier, harder brake on the
 * straight before the corner (exactly what a real driver does). On fast corners
 * a_lat is small, so almost the full circle is available for braking and they
 * are essentially unaffected - this slows the car ONLY where it is genuinely
 * grip-limited (the hairpins), instead of a flat budget that penalises
 * everywhere. Set a little above MAX_LATERAL_ACCEL_MS2 (steady cornering is kept
 * conservative; the combined circle has a touch more to give).
 *
 * 7.71 (from the sweep) sits just below the lateral budget: it pulls the worst
 * apex cross-track error down (the car brakes earlier for the hairpins and stops
 * washing the front wide) for little lap time, because it slows the hairpins and
 * leaves the fast corners alone. Raising it makes the circle never bind (more
 * apex drift); lowering it brakes ever earlier (smoother but slower). Runtime
 * tunable: g_GG_ACCEL_MS2 in shared/tunables.c. */

#define SPEED_PLAN_HORIZON_M 80.0f /* how far ahead to scan for corners, m */
#define SPEED_PLAN_STEPS     40    /* max waypoints in the scan */

/* Fraction of the speed-dependent downforce grip the ON-CAR planner is allowed
 * to use for corner-entry speed. The racing LINE is shaped for the full
 * downforce grip; the planner stays more conservative so the tracker keeps a
 * margin and does not wash wide and saturate the steering. 0 = old flat budget
 * (no downforce on the car); 1 = full downforce (no margin). Runtime tunable:
 * g_PLANNER_DOWNFORCE_FRAC in shared/tunables.c. */


/* ============================================================ */
/* DRIVER: throttle / brake controller                          */
/* ============================================================ */

/* All torque values here are MOTOR torque (Nm at the motor shaft). The gear
 * ratio is applied once inside the vehicle model when it turns motor torque
 * into wheel force. Do not pre-multiply by the gear ratio anywhere else. */
#define DRIVER_TORQUE_NM 117.6f /* max throttle torque, Nm (4 x 29.4) */
#define SPEED_KP_NM      800.0f /* throttle P-gain, Nm per m/s */
#define DRAG_FF_NM       0.259f /* drag feedforward, Nm per m/s */
#define DRIVER_BRAKE_NM  -38.8f /* max regen braking torque, Nm */
#define BRAKE_KP_NM      16.2f  /* brake P-gain, Nm per m/s */

/* Traction-circle reference. Throttle is scaled by sqrt(1 - (ay/ref)^2), so it
 * backs off while the car is loaded laterally and powers up as the corner
 * opens. Set below the ~13 m/s^2 peak so the cut bites during normal cornering
 * (using the true peak left the car powering wide on exit).
 *
 * 14.10 (from the sweep) lets the car power out hard with the high lateral
 * budget while staying clean: the corner-exit throttle cut was over-conservative
 * for how much grip the car actually has, leaving exit acceleration on the table
 * on a track that is ~80% corners. Lowering it powers out more weakly (slower
 * exits); raising it powers wide off the line (off-track). Runtime tunable:
 * g_LAT_GRIP_REF_MS2 in shared/tunables.c. */

/* Throttle integral, to trim the steady-state speed deficit on corner exit.
 * Kept small (P does the heavy lifting) and anti-wound: it only advances when
 * the throttle is free to respond, and resets on braking. */
#ifndef SPEED_KI_NM
#define SPEED_KI_NM 400.0f /* throttle I-gain, Nm per m/s per s */
#endif
#ifndef SPEED_I_MAX_NM
#define SPEED_I_MAX_NM 250.0f /* clamp on the integral contribution, Nm */
#endif


/* ============================================================ */
/* DRIVER: cone boundary safety net                             */
/* ============================================================ */

#define BOUNDARY_WARN_M      1.0f  /* steer away from a cone within this range */
#define BOUNDARY_CORR_GAIN   0.30f /* max boundary steer correction, rad */
#define BOUNDARY_SLOW_M      1.0f  /* slow down within this range of a cone */
#define BOUNDARY_SLOW_FACTOR 0.6f  /* speed floor multiplier at the cone face */


/* ============================================================ */
/* ECU: torque vectoring                                        */
/* ============================================================ */

/* Master proportional gain, Nm of torque bias per rad/s of yaw error (the
 * runtime tunable g_KP_YAW_DEFAULT in shared/tunables.c, also nudged live with
 * [ and ]). Ki and Kd are fractions of it, so the loop scales together. */

/* Integral and derivative gains, as a fraction of the master gain. I erases the
 * steady-state understeer; D damps the turn-in. */
#ifndef TV_KI_FRAC
#define TV_KI_FRAC 2.5f
#endif
#ifndef TV_KD_FRAC
#define TV_KD_FRAC 0.05f
#endif

/* Feedforward gain: bias per unit of (desired_yaw_rate * speed). Pre-loads the
 * differential from the cornering demand so the yaw moment is there before any
 * error develops. Runtime tunable: g_TV_KFF in shared/tunables.c. */

/* Cap on the bias the integral term alone can contribute, Nm. */
#define TV_I_MAX_NM 12.0f

/* Yaw-error deadband, rad/s. Errors smaller than this are treated as zero so
 * the differential does not chatter on sensor noise. */
#define TV_YAW_DEADBAND 0.03f

/* Understeer term in the desired-yaw reference:
 *   desired_yaw = v * tan(steer) / (WHEELBASE + TV_K_US * v^2)
 * The v^2 term bends the reference down to the yaw rate the car can actually
 * reach. 0 gives the plain kinematic estimate. */
#define TV_K_US 0.06f

/* Speed at which the gain scaling is neutral. Effective gain = Kp * (this / v),
 * so the yaw response stays consistent across the speed range. */
#define TV_SPEED_REF_MS 12.0f

/* Weight on the wheel-speed yaw estimate when fused with the IMU. 0 = IMU only,
 * 1 = wheel speeds only. Kept low because wheel speeds lose validity once a
 * tyre slips. */
#define TV_WHEEL_YAW_TRUST 0.25f

/* Motor torque limits, Nm. Negative is regenerative braking. */
#define MAX_MOTOR_TORQUE_NM 29.4f
#define MIN_MOTOR_TORQUE_NM -100.0f


/* ============================================================ */
/* Control loop timing                                          */
/* ============================================================ */

/* Fixed control period, s. The HIL host runs the loop at 100 Hz. Both the
 * driver and the ECU integral/derivative terms use this as their time base. */
#define CONTROL_DT_S 0.01f

#endif /* CONSTANTS_CONFIG_H */
