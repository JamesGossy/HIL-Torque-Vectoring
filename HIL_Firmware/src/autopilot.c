#include "../include/autopilot.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/*
 * autopilot.c
 *
 * Pure-pursuit steering controller.
 *
 * How pure pursuit works:
 *
 *   1. Find the lookahead point -- the waypoint that is roughly LOOKAHEAD_M
 *      metres ahead of the car along the track.
 *
 *   2. Calculate the angle from the car to the lookahead point in world space.
 *
 *   3. Calculate how far "off-angle" that point is relative to the car's
 *      current heading. This is the heading error.
 *
 *   4. Convert that heading error into a steering angle using the bicycle model
 *      formula: steer = atan(2 * wheelbase * sin(heading_error) / lookahead)
 *
 *   5. Clamp the steering to the physical limits of the car.
 *
 * Throttle is handled simply: if the car is below target speed, apply full
 * driver torque. If above, apply zero. This is an on/off throttle controller.
 * It is crude but good enough -- the physics will smooth it out.
 */

/* Maximum steering angle the car can physically achieve, radians (~20 degrees) */
#define MAX_STEER_RAD  0.35f


float autopilot_update(VehicleState *state, const Track *track)
{
    /* --- Find the best lookahead waypoint ---
     * Walk forward through the upcoming waypoints and pick the first one that
     * is at least LOOKAHEAD_M away from the car. If none is far enough (the
     * car has overshot them all), just use the current target waypoint. */

    float best_x = track->points[track->current_index].x;
    float best_y = track->points[track->current_index].y;
    int   count  = track->count;

    int i;
    for (i = 0; i < count; i++) {
        int idx = (track->current_index + i) % count;
        float dx = track->points[idx].x - state->x;
        float dy = track->points[idx].y - state->y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist >= LOOKAHEAD_M) {
            best_x = track->points[idx].x;
            best_y = track->points[idx].y;
            break;
        }
    }

    /* --- Compute the angle to the lookahead point in world space --- */
    float dx_world  = best_x - state->x;
    float dy_world  = best_y - state->y;
    float angle_to_target = atan2f(dy_world, dx_world);

    /* --- Heading error: difference between where car points and where target is --- */
    float heading_error = angle_to_target - state->heading;

    /* Normalise to [-pi, pi] */
    while (heading_error >  PI) heading_error -= 2.0f * PI;
    while (heading_error < -PI) heading_error += 2.0f * PI;

    /* --- Pure-pursuit steering formula ---
     * steer = atan( 2 * L * sin(heading_error) / lookahead )
     * where L is the wheelbase. This gives the steering angle needed to
     * follow a circular arc from the car to the lookahead point. */
    float lookahead = LOOKAHEAD_M;
    float steer = atanf(2.0f * WHEELBASE_M * sinf(heading_error) / lookahead);

    /* Clamp to physical steering limits */
    if (steer >  MAX_STEER_RAD) steer =  MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;

    state->steering = steer;

    /* --- Smooth speed controller: P feedback + drag feedforward ---
     * Feedforward keeps a steady torque at cruise so the wheel torque bars
     * always show a non-zero reading and TV differentials are visible. */
    float speed_error    = TARGET_SPEED_MS - state->velocity;
    float driver_torque  = DRAG_FF_NM * state->velocity   /* overcome drag at current speed */
                         + SPEED_KP_NM * speed_error;      /* P term for speed tracking */
    if (driver_torque < 0.0f)             driver_torque = 0.0f;
    if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;

    return driver_torque;
}
