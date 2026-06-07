#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "track.h"
#include "vehicle_model.h"

/*
 * motion_control.h
 *
 * The real-time "driver" layer.  Runs every simulation tick and outputs:
 *   - a steering angle written into state->steering
 *   - a driver torque demand (return value, Nm; positive = throttle,
 *     negative = regenerative braking)
 *
 *
 * STEERING — Pure Pursuit (geometric look-ahead)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * The previous Stanley law failed on the tight FSG hairpins: it is purely
 * reactive (heading error + cross-track feedback), so it turned in late, its
 * feedback then saturated at the steering limit, and the car ran several metres
 * wide while pinned at full lock.
 *
 * Pure Pursuit is a GEOMETRIC tracker.  It picks a target point on the racing
 * line a look-ahead distance Ld in front of the rear axle and computes the
 * single-arc steer angle that drives the rear axle through it:
 *
 *   delta = atan2( 2 * WHEELBASE * sin(alpha), Ld )
 *
 *   alpha   angle from the car heading to the look-ahead point (rad)
 *   Ld      look-ahead distance, adapted to speed:
 *             Ld = clamp(K_LOOKAHEAD * v, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M)
 *
 * A short Ld at low speed lets the car commit to the apex of a tight corner
 * instead of chording across it; a longer Ld at speed keeps the line smooth and
 * stable on the straights.  Because the steer command is computed directly from
 * the geometry of the corner ahead, it does not lag-then-saturate the way the
 * reactive Stanley feedback did.
 *
 * A gentle cone-repulsion term is added as a safety net: if the car comes
 * within BOUNDARY_WARN_M of a boundary cone it is nudged away from that wall.
 *
 *
 * SPEED — two-pass backward-sweep corner planner
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Pass 1 (forward): for each upcoming waypoint within SPEED_PLAN_HORIZON_M
 *   v_limit[i] = min(TARGET_SPEED_MS, sqrt(MAX_LATERAL_ACCEL_MS2 / kappa)).
 * Pass 2 (backward): propagate braking constraints from the furthest waypoint
 *   back to the car:  v[i] = min(v_limit[i], sqrt(v[i+1]^2 + 2*a_brake*ds)).
 * This models late-braking into corners and acceleration out of them.
 *
 * A steering-saturation throttle cut (see STEER_SAT_FRAC) closes the loop the
 * old controller was missing: when the wheel is near full lock the front tyres
 * are already spending their whole grip budget on turning, so any throttle just
 * powers the car wide.  Throttle is faded out as the steer command approaches
 * the limit, which is what stops the mid-hairpin "accelerate while pinned at
 * full lock and slide wide" failure.
 */


/* ---- Pure Pursuit gains ---- */
/*
 * Look-ahead distance adapts to speed:  Ld = clamp(K_LOOKAHEAD*v, MIN, MAX).
 *
 * K_LOOKAHEAD sets how far ahead (in seconds of travel) the tracker aims.  At
 * the ~2.5 m waypoint spacing of this track, a short floor (LOOKAHEAD_MIN_M) is
 * what lets the car bite into the tight hairpins: the target point sits just
 * past the apex, so the geometric arc bends hard into the corner instead of
 * chording across it.  The cap (LOOKAHEAD_MAX_M) keeps the line smooth at speed.
 *
 * A small cross-track trim (K_CTE_PP) is layered on Pure Pursuit so that, if the
 * car has been pushed off the line, it is actively pulled back rather than just
 * running parallel to it — Pure Pursuit alone has no restoring term once the
 * look-ahead point and the car drift onto the same offset line.
 */
/* These four gains are wrapped in #ifndef so a parameter sweep can override
 * them from the compiler command line (-DK_LOOKAHEAD=0.5f etc.) without editing
 * this file.  The values below are the tuned defaults. */
#ifndef K_LOOKAHEAD
#define K_LOOKAHEAD       0.45f   /* look-ahead time, s (Ld = this * v)       */
#endif
#ifndef LOOKAHEAD_MIN_M
#define LOOKAHEAD_MIN_M   2.8f    /* minimum look-ahead, m (tight corners)
                                   * — 2.8 won the sweep: fastest fully clean
                                   *   lap (32.0 s, 0 cone contacts).  Smaller
                                   *   floors shaved nothing off lap time but
                                   *   clipped 5–7 apex cones. */
#endif
#ifndef LOOKAHEAD_MAX_M
#define LOOKAHEAD_MAX_M   9.0f    /* maximum look-ahead, m (straights)        */
#endif
#ifndef K_CTE_PP
#define K_CTE_PP          0.35f   /* cross-track restoring trim, rad/m        */
#endif

/*
 * Steering limit (reference angle, rad).  The vehicle model multiplies this by
 * the Ackermann ratios (~0.20–0.26) to get the road-wheel angles, so the
 * reference angle is much larger than the actual wheel angle.  The OLD value of
 * 0.6 rad gave only ~0.13 rad at the wheel -> a kinematic minimum radius of
 * ~11 m.  The tightest FSG hairpin on this track is ~3.2 m radius, so at the old
 * limit the car PHYSICALLY could not follow the line and ran wide no matter how
 * good the tracker was.  1.7 rad reference -> ~0.39 rad at the loaded outer
 * wheel -> R_min ~3.8 m, enough to negotiate the hairpins with the speed plan
 * holding entry speed down. */
#define MAX_STEER_RAD  1.7f    /* steering reference limit (R_min ~3.8 m)  */

/* Throttle is faded out as the steer command rises above STEER_SAT_FRAC of the
 * limit: near full lock the front tyres are saturated turning, so throttle only
 * powers the car wide.  At/below the fraction throttle is unaffected; at the
 * limit it is fully cut. */
#define STEER_SAT_FRAC  0.7f

/*
 * Steering slew-rate limit (reference angle), rad/s.  The tracker can jump the
 * commanded angle discontinuously tick-to-tick (e.g. when the nearest-segment
 * projection steps to a new waypoint); a real driver / steering actuator
 * cannot.  The motion controller runs once per sim tick, so the per-tick step
 * is limited to MAX_STEER_RATE_RADS * MC_DT_S.  Raised from 4.0 to 8.0 rad/s
 * alongside the larger MAX_STEER_RAD so the wheel can still reach the (now
 * larger) lock fast enough to catch a tight hairpin turn-in.
 */
#ifndef MAX_STEER_RATE_RADS
#define MAX_STEER_RATE_RADS  8.0f
#endif
#define MC_DT_S              0.01f   /* sim tick period, s (100 Hz) -- matches DT */

/*
 * Nearest-segment search window around the controller's OWN progress index
 * (s_path_idx), not track->current_index.  The controller projects the car
 * onto the racing line every tick and advances its progress index with
 * continuity, so a slide never makes it skip past a corner.
 */
#define NEAREST_SEARCH_BACK   3    /* segments to look back                */
#define NEAREST_SEARCH_FWD   30    /* segments to look forward             */


/* ---- Speed planner ---- */
/*
 * MAX_BRAKE_DECEL_MS2 must not exceed what the drivetrain can actually deliver.
 * The driver brake command is clamped to DRIVER_BRAKE_NM = -38.8 Nm per motor.
 * After the 15.47 gear ratio that is ~600 Nm of wheel braking torque per wheel;
 * four wheels give ~2400 Nm total -> ~9 m/s^2 peak.  We plan for 5 m/s^2 so
 * the car commits to braking early enough rather than arriving at the corner
 * too fast (a planner that uses the peak decel tends to over-shoot).
 *
 */
#ifndef TARGET_SPEED_MS
#define TARGET_SPEED_MS        20.0f   /* cruise speed on straights, m/s   */
#endif
/*
 * MAX_LATERAL_ACCEL_MS2 is the corner-speed budget: v_corner = sqrt(a_lat/kappa).
 * The car's true peak is ~13 m/s^2.  This value was lowered from 7.0 to 4.0
 * after the per-wheel friction circle was added to the vehicle model: once the
 * tyres are honestly grip-limited, a 7.0 plan carried too much speed into the
 * corners and the car ran wide.  3.0 keeps the planned corner speed within what
 * the gripping tyres can hold while the Pure-Pursuit tracker corrects (it
 * minimised cone contact in the post-friction-circle sweep).
 *
 * NOTE: even at low corner speed the car cannot fully clean the single tightest
 * FSG hairpin (~3.2 m radius).  At full steering lock the front tyre is already
 * past its Pacejka grip peak (~12 deg slip), so it understeers there regardless
 * of speed — a genuine limit of THIS car's grip + steering geometry, not a
 * tuning error.  A feasibility-aware racing line (one that opens that apex to a
 * radius the car can actually hold) would be the proper fix; the current min-
 * curvature line leaves the car grazing that one apex.
 *
 * Wrapped in #ifndef so the parameter sweep can override it (-DMAX_LATERAL_ACCEL_MS2=8.0f). */
#ifndef MAX_LATERAL_ACCEL_MS2
#define MAX_LATERAL_ACCEL_MS2   3.0f   /* corner speed limit, m/s^2        */
#endif
#define MAX_BRAKE_DECEL_MS2     6.0f   /* braking look-ahead decel, m/s^2  */
#define SPEED_PLAN_HORIZON_M   80.0f   /* scan horizon for corners, metres */
#define SPEED_PLAN_STEPS        40     /* max waypoints to include in scan */


/* ---- Throttle / brake controller ---- */
/*
 * All torque constants below are MOTOR torque (Nm at the motor shaft).
 * The whole signal chain stays in motor torque: the driver demand, the ECU's
 * per-wheel WheelTorques outputs, and the motor clamps are all motor torque.
 * The gear ratio (GEAR_RATIO = 15.47) is applied once, inside the VEHICLE MODEL,
 * when it turns motor torque into wheel force (Fx = GEAR_RATIO * t / r).  The
 * ECU must NOT pre-multiply by the gear ratio -- that double-counts it and
 * saturates every motor at its clamp.
 *
 * BRAKE_KP_NM is deliberately high so the car actually follows the planned
 * deceleration into a corner (a soft gain under-brakes and arrives too fast).
 * LAT_GRIP_REF_MS2 is the traction-circle reference: throttle is scaled by
 * sqrt(1 - (ay/ref)^2) so it backs off while the car is loaded laterally and
 * only powers up as the corner opens (ay -> 0).  It is set BELOW the car's true
 * ~13 m/s^2 peak so the cut bites during normal cornering -- using the true peak
 * left ~90% of throttle flowing mid-corner, which powered the car wide on exit
 * (power understeer).  Lower it to defer throttle more; raise it for earlier
 * power-down.
 */
#define DRIVER_TORQUE_NM   117.6f   /* max throttle motor torque, Nm (4 x 29.4 Nm) */
#define SPEED_KP_NM          8.0f   /* throttle P-gain, Nm/(m/s).  Sized so a
                                      * typical corner-exit speed deficit
                                      * (~12 m/s below target) commands most of
                                      * the available motor torque, pushing the
                                      * loaded wheel toward its 29 Nm cap as the
                                      * corner opens and the grip budget frees up. */
#define DRAG_FF_NM           0.259f /* drag feedforward, Nm/(m/s)         */
#define DRIVER_BRAKE_NM    -38.8f   /* max regen braking motor torque, Nm */
#define BRAKE_KP_NM         16.2f   /* brake P-gain, Nm/(m/s)             */
#define LAT_GRIP_REF_MS2     8.0f   /* traction-circle reference, m/s^2 (below true peak; see above) */


/* ---- Cone boundary avoidance (safety net) ---- */
#define BOUNDARY_WARN_M      1.0f   /* steer correction starts this far from a cone  */
#define BOUNDARY_CORR_GAIN   0.30f  /* max boundary steering correction, rad         */
#define BOUNDARY_SLOW_M      1.0f   /* speed reduction starts this far from a cone   */
#define BOUNDARY_SLOW_FACTOR 0.6f   /* speed floor multiplier at the cone face       */


/*
 * Run one motion-control tick.
 * Writes state->steering and returns the driver torque demand.
 *
 * out_target_speed -- if non-NULL, receives the planner's commanded target
 *                     speed (m/s) for this tick.  Telemetry only; pass NULL if
 *                     not needed.
 */
float motion_control_update(VehicleState *state, const Track *track,
                            float *out_target_speed);

#endif /* MOTION_CONTROL_H */
