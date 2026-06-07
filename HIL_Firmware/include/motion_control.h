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
 * STEERING — true Stanley controller (nearest-point)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * The front axle is projected onto the NEAREST segment of the racing line.
 * The control law is the classic Stanley method:
 *
 *   steer = heading_error + atan( K_CTE * cross_track_error / (v + K_SOFT) )
 *
 *   heading_error      angle between the car heading and the tangent of the
 *                      nearest path segment.  Aligns the car with the path.
 *   cross_track_error  signed perpendicular distance from the front axle to
 *                      the nearest path segment.  Pulls the axle back onto
 *                      the line.
 *
 * Measuring both terms against the NEAREST segment (rather than a far
 * look-ahead chord) is what makes the car track the racing line tightly
 * instead of cutting across corner apexes.
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
 */


/* ---- Stanley gains ---- */
#define K_CTE          3.0f    /* cross-track error gain, rad/m            */
#define K_SOFT         2.5f    /* softening speed, m/s                     */

/*
 * Curvature feedforward gain (dimensionless, ~1.0 = full kinematic value).
 * Pure Stanley is reactive and always lags into a corner, so it turns in late
 * and runs wide on tight apexes.  Feeding forward the path curvature
 * (delta_ff = K_CURV_FF * atan(WHEELBASE * kappa)) pre-aligns the wheel with the
 * corner so the reactive terms only trim the residual. */
#define K_CURV_FF      1.15f
#define MAX_STEER_RAD  0.6f    /* steering limit (~34 deg; R_min ~2.3 m)   */

/*
 * Steering slew-rate limit (road-wheel angle), rad/s.  The Stanley law can jump
 * the commanded angle discontinuously tick-to-tick; a real driver / steering
 * actuator cannot.  4 rad/s at the road wheel sweeps the full +/-0.6 rad lock in
 * ~0.3 s -- still a quick driver, but no longer visibly snapping.  The motion
 * controller runs once per sim tick, so the per-tick step is limited to
 * MAX_STEER_RATE_RADS * MC_DT_S.
 */
#define MAX_STEER_RATE_RADS  4.0f
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
 * MAX_LATERAL_ACCEL_MS2 is kept well below the car's true grip (~13 m/s^2)
 * so there is margin for the Stanley tracker to correct, and because the
 * discrete Menger curvature sampled on ~3 m waypoint spacing under-reads tight
 * hairpin apexes — a conservative value keeps the planned corner speed safe.
 */
/*
 * MAX_LATERAL_ACCEL_MS2 is kept well below the car's true grip (~13 m/s^2)
 * for two reasons: it leaves margin for the Stanley tracker to correct, and
 * the discrete Menger curvature sampled on ~3 m waypoint spacing under-reads
 * the true apex sharpness of tight hairpins, so a conservative value keeps the
 * planned corner speed safe there.
 */
#define TARGET_SPEED_MS        20.0f   /* cruise speed on straights, m/s   */
#define MAX_LATERAL_ACCEL_MS2   3.5f   /* corner speed limit, m/s^2        */
#define MAX_BRAKE_DECEL_MS2     5.0f   /* braking look-ahead decel, m/s^2  */
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
