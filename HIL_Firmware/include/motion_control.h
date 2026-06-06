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
 * STEERING — Stanley controller
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * steer = heading_error + atan( K_CTE * cross_track_error / (v + K_SOFT) )
 *
 *   heading_error     difference between the car's heading and the direction
 *                     of the nearest reference segment.  Points the car along
 *                     the path.
 *   cross_track_error signed perpendicular distance from the front axle to
 *                     the reference segment.  The atan term adds extra steer
 *                     to pull the axle back onto the line.
 *   K_CTE             cross-track gain.  Higher = snappier return but more
 *                     oscillation.
 *   K_SOFT            softening constant.  Reduces gain at low speed to avoid
 *                     overcorrection at a crawl.
 *
 * The reference segment is found with a speed-adaptive lookahead:
 *   ld = clamp( v * LOOKAHEAD_TIME_S, LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M )
 * Shorter at low speed (tight corners), longer at high speed (straights).
 *
 *
 * SPEED — path-distance corner-speed planner
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Scans up to SPEED_PLAN_HORIZON_M metres ahead along the waypoints.
 * At each upcoming waypoint the Menger curvature κ is computed.
 * The maximum speed to arrive at the corner apex at the safe speed is:
 *   v_now_max = sqrt( v_corner^2 + 2 * MAX_BRAKE_DECEL_MS2 * path_dist )
 * The most restrictive value over all upcoming corners is the target speed.
 * A P-throttle + drag feedforward controller tracks the target when below it;
 * a P-brake controller tracks it when above.
 */


/* ---- Stanley gains ---- */
#define K_CTE          0.8f    /* cross-track error gain, rad/m            */
#define K_SOFT         4.0f    /* softening speed, m/s                     */
#define MAX_STEER_RAD  0.5f    /* physical steering limit (~28.6 deg)      */

/* Speed-adaptive lookahead for reference-segment selection */
#define LOOKAHEAD_TIME_S   0.4f   /* seconds of travel ahead to look       */
#define LOOKAHEAD_MIN_M    3.0f   /* minimum lookahead distance, metres    */
#define LOOKAHEAD_MAX_M   10.0f   /* maximum lookahead distance, metres    */


/* ---- Speed planner ---- */
/*
 * MAX_LATERAL_ACCEL_MS2: M25 Pacejka peak ≈ 1.14 g at low speed, rising to
 * ~1.5 g at 14 m/s once downforce is included.  11 m/s² gives a ~20% margin
 * against the low-speed limit so the planner brakes early enough.
 */
#define TARGET_SPEED_MS        14.0f   /* cruise speed on straights, m/s   */
#define MAX_LATERAL_ACCEL_MS2  11.0f   /* corner speed limit, m/s^2        */
#define MAX_BRAKE_DECEL_MS2     9.0f   /* braking look-ahead decel, m/s^2  */
#define SPEED_PLAN_HORIZON_M   60.0f   /* scan horizon for corners, metres */


/* ---- Throttle / brake controller ---- */
/*
 * DRAG_FF_NM: M25 quadratic drag at 14 m/s ≈ 228 N → 58 Nm total →
 * 58/14 ≈ 4 Nm/(m/s).  Slightly conservative to avoid oscillation.
 */
#define DRIVER_TORQUE_NM   800.0f   /* max throttle torque, Nm            */
#define SPEED_KP_NM         60.0f   /* throttle P-gain, Nm/(m/s)          */
#define DRAG_FF_NM           4.0f   /* drag feedforward, Nm/(m/s)         */
#define DRIVER_BRAKE_NM   -600.0f   /* max regen braking torque, Nm       */
#define BRAKE_KP_NM         80.0f   /* brake P-gain, Nm/(m/s)             */


/*
 * Run one motion-control tick.
 * Writes state->steering and returns the driver torque demand.
 */
float motion_control_update(VehicleState *state, const Track *track);

#endif /* MOTION_CONTROL_H */
