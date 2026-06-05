#include "../include/vehicle_model.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/*
 * vehicle_model.c
 *
 * This is the "virtual car". Every 10 ms (or whatever dt you use), we take
 * the torque commands from the ECU and the current state, and compute where
 * the car is and how it is moving at the next moment in time.
 *
 * The physics here are a simplified bicycle model. Here is the intuition:
 *
 *   1. Total drive force comes from the sum of all four wheel torques divided
 *      by the wheel radius. F = (T_fl + T_fr + T_rl + T_rr) / R
 *
 *   2. A linear drag force opposes motion: F_drag = DRAG_COEFF * velocity
 *
 *   3. Net force accelerates the car: a = (F_drive - F_drag) / MASS
 *
 *   4. The yaw rate (rotation speed) comes from the kinematic bicycle model:
 *      yaw_rate = velocity * tan(steering) / wheelbase
 *      This is exact for low speed. At higher speeds a real tyre model would
 *      give a more accurate answer, but this is good enough for a teaching tool.
 *
 *   5. The heading integrates yaw_rate over time.
 *
 *   6. The position integrates velocity in the direction of heading over time.
 *
 * A left/right torque difference (from torque vectoring) creates a yaw moment:
 *   yaw_moment = (T_right - T_left) * track_width / (2 * wheel_radius)
 *   This moment adds to the yaw acceleration: yaw_accel = moment / yaw_inertia
 */


/* Approximate yaw moment of inertia for a small race car, kg*m^2 */
#define YAW_INERTIA_KGM2  300.0f

/* Maximum steering angle, radians (about 20 degrees) */
#define MAX_STEER_RAD  0.35f


void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading)
{
    s->x          = start_x;
    s->y          = start_y;
    s->heading    = start_heading;
    s->velocity   = 0.0f;
    s->yaw_rate   = 0.0f;
    s->slip_angle = 0.0f;
    s->steering   = 0.0f;
}


void vehicle_model_update(VehicleState *s, const WheelTorques *t, float dt)
{
    /* --- 1. Total drive force from all four wheels --- */
    float total_torque = t->fl + t->fr + t->rl + t->rr;
    float drive_force  = total_torque / WHEEL_RADIUS_M;

    /* --- 2. Drag force (opposes motion) --- */
    float drag_force = DRAG_COEFF * s->velocity;

    /* --- 3. Net longitudinal acceleration --- */
    float net_force = drive_force - drag_force;
    float accel     = net_force / MASS_KG;

    /* --- 4. Left/right torque difference creates a yaw moment ---
     * More torque on the right side makes the car turn left (yaw rate increases).
     * More torque on the left side makes the car turn right (yaw rate decreases). */
    float torque_diff_front = t->fr - t->fl;
    float torque_diff_rear  = t->rr - t->rl;
    float yaw_moment = (torque_diff_front + torque_diff_rear)
                       * (TRACK_WIDTH_M / 2.0f) / WHEEL_RADIUS_M;
    float yaw_accel  = yaw_moment / YAW_INERTIA_KGM2;

    /* --- 5. Kinematic yaw rate from steering angle ---
     * At speed, the kinematic model predicts the yaw rate from geometry alone. */
    float kinematic_yaw_rate = 0.0f;
    if (s->velocity > 0.5f) {
        kinematic_yaw_rate = s->velocity * tanf(s->steering) / WHEELBASE_M;
    }

    /* Blend toward the kinematic yaw rate, and add the TV-induced yaw acceleration.
     * The blending constant (5.0) sets how quickly the car's yaw rate follows the
     * kinematic target. Increase it for a stiffer feel, decrease for a lazier feel. */
    float yaw_rate_error = kinematic_yaw_rate - s->yaw_rate;
    s->yaw_rate += (5.0f * yaw_rate_error + yaw_accel) * dt;

    /* --- 6. Integrate heading and position --- */
    s->heading  += s->yaw_rate * dt;
    s->velocity += accel * dt;

    /* Clamp speed to physical limits */
    if (s->velocity < 0.0f)       s->velocity = 0.0f;
    if (s->velocity > MAX_SPEED_MS) s->velocity = MAX_SPEED_MS;

    /* Update world position using current heading */
    s->x += s->velocity * cosf(s->heading) * dt;
    s->y += s->velocity * sinf(s->heading) * dt;

    /* --- 7. Body slip angle ---
     * This is the angle between where the car is pointing and where it is moving.
     * At low speed it is near zero. At the limit it becomes large (drifting). */
    if (s->velocity > 0.5f) {
        float vy = s->yaw_rate * WHEELBASE_M * 0.5f;  /* approximate lateral velocity */
        s->slip_angle = atanf(vy / s->velocity);
    } else {
        s->slip_angle = 0.0f;
    }

    /* Keep heading in [-pi, pi] range to avoid it growing without bound */
    while (s->heading >  PI) s->heading -= 2.0f * PI;
    while (s->heading < -PI) s->heading += 2.0f * PI;
}
