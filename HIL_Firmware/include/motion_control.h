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
#define K_CTE          1.5f    /* cross-track error gain, rad/m            */
#define K_SOFT         2.5f    /* softening speed, m/s                     */
#define MAX_STEER_RAD  0.6f    /* steering limit (~34 deg; R_min ~2.3 m)   */

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
 * MAX_BRAKE_DECEL_MS2 is intentionally set BELOW the physically achievable
 * brake deceleration (~9 m/s^2 from -600 Nm regen) so the planner commits to
 * braking early enough rather than overshooting the corner-entry speed.
 */
/*
 * MAX_BRAKE_DECEL_MS2 must not exceed what the drivetrain can actually deliver.
 * The ECU clamps each motor to -100 Nm of regen, so total braking torque is
 * limited to -400 Nm -> ~6 m/s^2 decel.  We plan for 5 m/s^2 so the car commits
 * to braking early enough rather than arriving at the corner too fast.
 */
/*
 * MAX_LATERAL_ACCEL_MS2 is kept well below the car's true grip (~13 m/s^2)
 * for two reasons: it leaves margin for the Stanley tracker to correct, and
 * the discrete Menger curvature sampled on ~3 m waypoint spacing under-reads
 * the true apex sharpness of tight hairpins, so a conservative value keeps the
 * planned corner speed safe there.
 */
#define TARGET_SPEED_MS        12.0f   /* cruise speed on straights, m/s   */
#define MAX_LATERAL_ACCEL_MS2   5.5f   /* corner speed limit, m/s^2        */
#define MAX_BRAKE_DECEL_MS2     5.0f   /* braking look-ahead decel, m/s^2  */
#define SPEED_PLAN_HORIZON_M   80.0f   /* scan horizon for corners, metres */
#define SPEED_PLAN_STEPS        40     /* max waypoints to include in scan */


/* ---- Throttle / brake controller ---- */
/*
 * BRAKE_KP_NM is deliberately high so the car actually follows the planned
 * deceleration into a corner (a soft gain under-brakes and arrives too fast).
 * LAT_GRIP_REF_MS2 is the peak lateral acceleration the car can sustain
 * (with downforce, the M25 reaches ~13 m/s^2); throttle is scaled down by the
 * remaining traction-circle budget so the car does not power-understeer out of
 * a corner before it has opened up.
 */
#define DRIVER_TORQUE_NM   800.0f   /* max throttle torque, Nm            */
#define SPEED_KP_NM         60.0f   /* throttle P-gain, Nm/(m/s)          */
#define DRAG_FF_NM           4.0f   /* drag feedforward, Nm/(m/s)         */
#define DRIVER_BRAKE_NM   -600.0f   /* max regen braking torque, Nm       */
#define BRAKE_KP_NM        250.0f   /* brake P-gain, Nm/(m/s)             */
#define LAT_GRIP_REF_MS2    13.0f   /* peak lateral accel for traction circle */


/* ---- Cone boundary avoidance (safety net) ---- */
#define BOUNDARY_WARN_M      1.0f   /* steer correction starts this far from a cone  */
#define BOUNDARY_CORR_GAIN   0.30f  /* max boundary steering correction, rad         */
#define BOUNDARY_SLOW_M      1.0f   /* speed reduction starts this far from a cone   */
#define BOUNDARY_SLOW_FACTOR 0.6f   /* speed floor multiplier at the cone face       */


/*
 * Run one motion-control tick.
 * Writes state->steering and returns the driver torque demand.
 */
float motion_control_update(VehicleState *state, const Track *track);

#endif /* MOTION_CONTROL_H */
