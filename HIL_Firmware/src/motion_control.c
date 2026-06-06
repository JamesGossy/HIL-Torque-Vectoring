#include "../include/motion_control.h"
#include <math.h>
#define BRAKE_ENGAGE_MS 500 
static const float PI = 3.14159265358979323846f;


/* ---- Menger curvature helper ---- */

static float menger_curvature(float ax, float ay,
                               float bx, float by,
                               float cx, float cy)
{
    float abx = bx-ax, aby = by-ay;
    float bcx = cx-bx, bcy = cy-by;
    float cax = ax-cx, cay = ay-cy;
    float ab  = sqrtf(abx*abx + aby*aby);
    float bc  = sqrtf(bcx*bcx + bcy*bcy);
    float ca  = sqrtf(cax*cax + cay*cay);
    float crs = fabsf(abx*bcy - aby*bcx);
    if (ab < 0.01f || bc < 0.01f || ca < 0.01f || crs < 1e-6f) return 0.0f;
    return 2.0f * crs / (ab * bc * ca);
}


/* ---- Speed planner ---- */

/*
 * Scan upcoming waypoints for curvature and back-propagate along the path
 * to find the maximum safe speed this tick.
 */
static float plan_target_speed(const Track *track)
{
    float v_plan = TARGET_SPEED_MS;
    int   count  = track->count;
    float path_s = 0.0f;
    int   i;

    for (i = 1; i < count; i++) {
        int i0 = (track->current_index + i - 1) % count;
        int i1 = (track->current_index + i    ) % count;
        int i2 = (track->current_index + i + 1) % count;

        float seg_dx = track->points[i1].x - track->points[i0].x;
        float seg_dy = track->points[i1].y - track->points[i0].y;
        path_s += sqrtf(seg_dx*seg_dx + seg_dy*seg_dy);

        if (path_s > SPEED_PLAN_HORIZON_M) break;

        float kappa = menger_curvature(
            track->points[i0].x, track->points[i0].y,
            track->points[i1].x, track->points[i1].y,
            track->points[i2].x, track->points[i2].y);

        if (kappa < 1e-4f) continue;

        float v_corner   = sqrtf(MAX_LATERAL_ACCEL_MS2 / kappa);
        if (v_corner >= TARGET_SPEED_MS) continue;

        float v_approach = sqrtf(v_corner*v_corner + 2.0f * MAX_BRAKE_DECEL_MS2 * path_s);
        if (v_approach < v_plan) v_plan = v_approach;
    }

    return v_plan;
}


/* ---- Stanley steering ---- */

/*
 * Walk forward along the waypoints until the accumulated path distance
 * exceeds the lookahead, then return the index of the segment endpoint.
 */
static int find_reference_segment(const Track *track, float lookahead)
{
    int   count  = track->count;
    float path_s = 0.0f;
    int   best   = (track->current_index + 1) % count;
    int   i;

    for (i = 1; i < count; i++) {
        int i0 = (track->current_index + i - 1) % count;
        int i1 = (track->current_index + i    ) % count;

        float dx = track->points[i1].x - track->points[i0].x;
        float dy = track->points[i1].y - track->points[i0].y;
        path_s += sqrtf(dx*dx + dy*dy);

        best = i1;
        if (path_s >= lookahead) break;
    }
    return best;
}


/* ---- Public update ---- */

float motion_control_update(VehicleState *state, const Track *track)
{
    int   count = track->count;
    float vx    = state->velocity;

    /* ------------------------------------------------------------------ */
    /* STEERING                                                             */
    /* ------------------------------------------------------------------ */

    float lookahead = vx * LOOKAHEAD_TIME_S;
    if (lookahead < LOOKAHEAD_MIN_M) lookahead = LOOKAHEAD_MIN_M;
    if (lookahead > LOOKAHEAD_MAX_M) lookahead = LOOKAHEAD_MAX_M;

    int   ref  = find_reference_segment(track, lookahead);
    int   prev = (ref - 1 + count) % count;

    float seg_x  = track->points[ref].x  - track->points[prev].x;
    float seg_y  = track->points[ref].y  - track->points[prev].y;
    float seg_len = sqrtf(seg_x*seg_x + seg_y*seg_y);

    float ref_heading = (seg_len > 0.01f) ? atan2f(seg_y, seg_x) : state->heading;

    float heading_error = ref_heading - state->heading;
    while (heading_error >  PI) heading_error -= 2.0f * PI;
    while (heading_error < -PI) heading_error += 2.0f * PI;

    /* Signed cross-track error at the front axle */
    float fa_x = state->x + CG_TO_FRONT_M * cosf(state->heading);
    float fa_y = state->y + CG_TO_FRONT_M * sinf(state->heading);

    float to_fa_x = fa_x - track->points[prev].x;
    float to_fa_y = fa_y - track->points[prev].y;

    float inv_len = 1.0f / (seg_len + 0.01f);
    float cte = -(to_fa_x * (-seg_y * inv_len) + to_fa_y * (seg_x * inv_len));

    float steer = heading_error + atanf(K_CTE * cte / (vx + K_SOFT));

    if (steer >  MAX_STEER_RAD) steer =  MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;

    state->steering = steer;

    /* ------------------------------------------------------------------ */
    /* SPEED                                                                */
    /* ------------------------------------------------------------------ */

    float target_speed = plan_target_speed(track);
    float speed_error  = target_speed - vx;
    float driver_torque;

    if (speed_error >= 0.0f) {
        driver_torque = DRAG_FF_NM * vx + SPEED_KP_NM * speed_error;
        if (driver_torque < 0.0f)             driver_torque = 0.0f;
        if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;
    } else {
        driver_torque = BRAKE_KP_NM * speed_error;
        if (driver_torque < DRIVER_BRAKE_NM) driver_torque = DRIVER_BRAKE_NM;
        if (driver_torque > 0.0f)            driver_torque = 0.0f;
    }

    return driver_torque;
}
