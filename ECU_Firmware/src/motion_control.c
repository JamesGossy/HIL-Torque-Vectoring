// The driver, now on the ECU. Steers with a kinematic feedforward plus Stanley
// feedback, plans corner speed with a two-pass friction-circle planner, and sets
// driver torque. It drives on an estimated pose (CtrlPose) and the ECU's own map
// (EcuMap), never the HIL vehicle state. Gains are g_* globals in
// shared/tunables.c; physical constants in vehicle_config.h.

#include "../include/motion_control.h"
#include "../../shared/grip_model.h"
#include "../../shared/tunables.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/* ---- steering ---- */

static float s_Kus       = 0.0f; // understeer gradient, derived once from config
static int s_steer_ready = 0;

// Derive the understeer gradient from the tyre model.
static void steer_init(void)
{
    const float g = 9.81f;
    float Wf      = MASS_KG * g * CG_TO_REAR_M / WHEELBASE_M; // front static load
    float Wr      = MASS_KG * g * CG_TO_FRONT_M / WHEELBASE_M;
    float Cf      = TYRE_D * Wf * fabsf(TYRE_C) * TYRE_B; // Pacejka slope at alpha=0
    float Cr      = TYRE_D * Wr * fabsf(TYRE_C) * TYRE_B;
    s_Kus         = (MASS_KG / (2.0f * WHEELBASE_M)) * (CG_TO_REAR_M / Cf - CG_TO_FRONT_M / Cr);
    s_steer_ready = 1;
}

// Return the understeer gradient (rad per m/s^2 of lateral accel).
float driver_understeer_gradient(void)
{
    if (!s_steer_ready) steer_init();
    return s_Kus;
}

// Steering reference (rad) from path curvature and tracking error. e1 cross-track (+ve car left of path), e2 heading error, path_kappa signed curvature (+ve left bend).
float steer_command(float vx, float e1, float e2, float yaw_rate, float path_kappa)
{
    if (!s_steer_ready) steer_init();

    // feedforward from curvature
    float delta_ff_wheel = WHEELBASE_M * path_kappa + s_Kus * vx * vx * path_kappa; // curvature + understeer feedforward
    float delta_ff       = delta_ff_wheel / ACK_NOMINAL;

    // stanley feedback on heading and cross-track
    float delta_head = -e2;
    float delta_cte  = atan2f(g_K_STANLEY * (-e1), 4.0f); // speed-scaled, not 1/v, so it keeps authority in fast corners

    // yaw-rate damping
    float yaw_err   = yaw_rate - vx * path_kappa; // damping the kinematic Stanley law lacks, stops wash-wide at speed
    float delta_yaw = -g_K_DAMP * yaw_err;

    return delta_ff + delta_head + delta_cte + delta_yaw;
}

/* ---- geometry helpers ---- */

static float menger_curvature(float ax, float ay, float bx, float by, float cx, float cy)
{
    float abx = bx - ax, aby = by - ay;
    float bcx = cx - bx, bcy = cy - by;
    float cax = ax - cx, cay = ay - cy;
    float ab  = sqrtf(abx * abx + aby * aby);
    float bc  = sqrtf(bcx * bcx + bcy * bcy);
    float ca  = sqrtf(cax * cax + cay * cay);
    float crs = fabsf(abx * bcy - aby * bcx);
    if (ab < 0.01f || bc < 0.01f || ca < 0.01f || crs < 1e-6f) return 0.0f;
    return 2.0f * crs / (ab * bc * ca);
}

static float wrap_pi(float a)
{
    while (a > PI)
        a -= 2.0f * PI;
    while (a < -PI)
        a += 2.0f * PI;
    return a;
}

/* ---- speed planner ---- */

// Available braking decel (m/s^2) at speed v on a corner of curvature kappa, from the friction circle.
static float brake_decel_avail(float v, float kappa)
{
    float gg_acc = g_GRIP_USE * peak_lat(v); // same grip budget the car corners at, grows with downforce
    float a_lat  = v * v * kappa;
    float gg     = gg_acc * gg_acc;
    float a_lat2 = a_lat * a_lat;
    float a_lon  = (a_lat2 < gg) ? sqrtf(gg - a_lat2) : 0.0f;
    if (a_lon > g_MAX_BRAKE_DECEL_MS2) a_lon = g_MAX_BRAKE_DECEL_MS2;
    if (a_lon < 0.5f) a_lon = 0.5f; // keep the sweep progressing
    return a_lon;
}

// Set a speed cap per upcoming waypoint from curvature, then back-propagate braking under the friction circle.
static float plan_target_speed(const CtrlPose *pose, const EcuMap *map, int start_idx)
{
    float v_limit[SPEED_PLAN_STEPS_CAP];
    float seg_len[SPEED_PLAN_STEPS_CAP]; // distance from point i to i+1
    float seg_k[SPEED_PLAN_STEPS_CAP];   // curvature at point i, for the GG budget
    int count    = map->count;
    int n        = 0;
    float path_s = 0.0f;
    int i;

    for (i = 0; i < g_SPEED_PLAN_STEPS; i++) { // forward pass, scan depth is g_SPEED_PLAN_STEPS
        int ccur = (start_idx + i) % count;

        float kappa = 0.0f; // max over several stencil spacings so a sharp apex is not under-read
        for (int d = 1; d <= 3; d++) {
            int cprev = (start_idx + i - d + count) % count;
            int cnext = (start_idx + i + d) % count;
            float k   = menger_curvature(map->points[cprev].x, map->points[cprev].y,
                map->points[ccur].x, map->points[ccur].y, map->points[cnext].x,
                map->points[cnext].y);
            if (k > kappa) kappa = k;
        }
        int cnext = (start_idx + i + 1) % count;

        float budget = g_GRIP_USE * PEAK_LAT_FLAT; // GRIP_USE < 1 keeps the tracker off the limit, apex_speed folds in downforce
        v_limit[n]   = apex_speed(budget, kappa, TARGET_SPEED_MS);
        seg_k[n]     = kappa;

        float dx   = map->points[cnext].x - map->points[ccur].x;
        float dy   = map->points[cnext].y - map->points[ccur].y;
        seg_len[n] = sqrtf(dx * dx + dy * dy);
        n++;

        path_s += seg_len[n - 1];
        if (path_s > g_SPEED_PLAN_HORIZON_M) break;
    }

    if (n == 0) return TARGET_SPEED_MS;

    // Backward pass: decel over segment i uses the friction circle at the downstream end (i+1), where braking room is smallest.
    float v_sweep = v_limit[n - 1];
    for (i = n - 2; i >= 0; i--) {
        float a_brake      = brake_decel_avail(v_sweep, seg_k[i + 1]);
        float v_can_arrive = sqrtf(v_sweep * v_sweep + 2.0f * a_brake * seg_len[i]);
        v_sweep            = (v_limit[i] < v_can_arrive) ? v_limit[i] : v_can_arrive;
    }

    // Final step: braking from the car to the first upcoming waypoint.
    {
        int wp0       = start_idx % count;
        float ddx     = map->points[wp0].x - pose->x;
        float ddy     = map->points[wp0].y - pose->y;
        float d_now   = sqrtf(ddx * ddx + ddy * ddy);
        float a_brake = brake_decel_avail(v_sweep, seg_k[0]);
        float v_now   = sqrtf(v_sweep * v_sweep + 2.0f * a_brake * d_now);
        if (v_now < TARGET_SPEED_MS) return v_now;
    }

    return TARGET_SPEED_MS;
}


// Find the racing-line segment nearest (px, py) near center_idx; returns its start index and writes signed cross-track error to *out_cte.
static int find_nearest_segment(const EcuMap *map, float px, float py, int center_idx, float *out_cte)
{
    int count      = map->count;
    int best       = center_idx;
    float best_d2  = 1e18f;
    float best_cte = 0.0f;
    int k;

    for (k = -g_NEAREST_SEARCH_BACK; k <= g_NEAREST_SEARCH_FWD; k++) {
        int i0 = ((center_idx + k) % count + count) % count;
        int i1 = (i0 + 1) % count;

        float ax = map->points[i0].x, ay = map->points[i0].y;
        float ex   = map->points[i1].x - ax;
        float ey   = map->points[i1].y - ay;
        float len2 = ex * ex + ey * ey;
        if (len2 < 1e-6f) continue;

        float t = ((px - ax) * ex + (py - ay) * ey) / len2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float cx = ax + t * ex;
        float cy = ay + t * ey;
        float dx = px - cx, dy = py - cy;
        float d2 = dx * dx + dy * dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            best    = i0;
            float inv = 1.0f / sqrtf(len2); // signed CTE, +ve when right of the segment direction
            best_cte  = (dx * (-ey) + dy * ex) * inv * -1.0f;
        }
    }

    if (out_cte) *out_cte = best_cte;
    return best;
}


// Max curvature (1/m) over the waypoints within span metres ahead of start_idx.
static float lookahead_curvature(const EcuMap *map, int start_idx, float span)
{
    int count  = map->count;
    float acc  = 0.0f;
    float kmax = 0.0f;
    int i;

    for (i = 0; i < count; i++) {
        int ccur = (start_idx + i) % count;
        for (int d = 1; d <= 3; d++) {
            int cprev = (start_idx + i - d + count) % count;
            int cnext = (start_idx + i + d) % count;
            float k   = menger_curvature(map->points[cprev].x, map->points[cprev].y,
                map->points[ccur].x, map->points[ccur].y, map->points[cnext].x,
                map->points[cnext].y);
            if (k > kmax) kmax = k;
        }
        int nxt  = (start_idx + i + 1) % count;
        float dx = map->points[nxt].x - map->points[ccur].x;
        float dy = map->points[nxt].y - map->points[ccur].y;
        acc += sqrtf(dx * dx + dy * dy);
        if (acc > span) break;
    }
    return kmax;
}


static int s_path_idx         = -1;    // progress index, -1 means uninitialised; independent of map->current_index
static float s_speed_integral = 0.0f;  // throttle integral state, Nm

// Reset the driver's internal state before an independent run.
void motion_control_reset(void)
{
    s_path_idx       = -1;
    s_speed_integral = 0.0f;
}


// Run one control tick: compute steering, plan target speed, and return driver torque.
float motion_control_update(
    const CtrlPose *pose, const EcuMap *map, float *out_steering_rad, float *out_target_speed)
{
    int count = map->count;
    float vx  = pose->vx;

    if (s_path_idx < 0 || s_path_idx >= count) s_path_idx = map->current_index;

    // 1. project the front axle onto the racing line, read cross-track error, advance progress index
    float fa_x = pose->x + CG_TO_FRONT_M * cosf(pose->heading);
    float fa_y = pose->y + CG_TO_FRONT_M * sinf(pose->heading);

    float cte;
    int i0 = find_nearest_segment(map, fa_x, fa_y, s_path_idx, &cte);

    s_path_idx = i0;

    // 2. Stanley + feedforward steering command
    float steer;
    {
        int i1   = (i0 + 1) % count;
        float ph = atan2f(
            map->points[i1].y - map->points[i0].y, map->points[i1].x - map->points[i0].x);
        float e2 = wrap_pi(pose->heading - ph);
        float e1 = -cte; // error model takes +e1 to the left, cte is +ve when right
        float kp = lookahead_curvature(map, i0, 1.0f); // curvature at car
        int i2    = (i0 + 2) % count; // sign of curvature from heading change across the segment, + when turning left
        float ph2 = atan2f(
            map->points[i2].y - map->points[i1].y, map->points[i2].x - map->points[i1].x);
        float dpsi         = wrap_pi(ph2 - ph);
        float kappa_signed = (dpsi >= 0.0f ? kp : -kp);

        steer = steer_command(vx, e1, e2, pose->yaw_rate, kappa_signed);
    }

    if (steer > g_MAX_STEER_RAD) steer = g_MAX_STEER_RAD;
    if (steer < -g_MAX_STEER_RAD) steer = -g_MAX_STEER_RAD;

    // Slew-rate limit: cap how far the commanded angle moves in one tick.
    {
        float max_step = g_MAX_STEER_RATE_RADS * CONTROL_DT_S;
        float dsteer   = steer - pose->steering;
        if (dsteer > max_step) steer = pose->steering + max_step;
        if (dsteer < -max_step) steer = pose->steering - max_step;
    }

    if (out_steering_rad) *out_steering_rad = steer;

    // 3. two-pass speed planner: forward apex caps then backward brake propagation
    float target_speed = plan_target_speed(pose, map, s_path_idx);

    if (out_target_speed) *out_target_speed = target_speed;

    float speed_error = target_speed - vx;
    float driver_torque;

    // 4. throttle or brake torque demand
    if (speed_error >= 0.0f) {
        // Traction circle: cut throttle by lateral grip in use so the car powers up only as the corner opens. Uses lagged ay_filt.
        float lat_ratio = fabsf(pose->ay_filt) / peak_lat(vx);
        if (lat_ratio > 1.0f) lat_ratio = 1.0f;
        float grip_factor = sqrtf(1.0f - lat_ratio * lat_ratio);

        // Steering-saturation cut: fade throttle from g_STEER_SAT_FRAC of lock to zero near full lock.
        float steer_factor = 1.0f;
        float steer_frac   = fabsf(steer) / g_MAX_STEER_RAD;
        if (steer_frac > g_STEER_SAT_FRAC) {
            float over = (steer_frac - g_STEER_SAT_FRAC) / (1.0f - g_STEER_SAT_FRAC);
            if (over > 1.0f) over = 1.0f;
            steer_factor = 1.0f - over;
        }

        float drag_force = 0.5f * AIR_DENSITY * CDA * AERO_AREA * vx * vx; // drag feedforward, derived from aero
        float drag_ff    = drag_force * WHEEL_RADIUS_M / GEAR_RATIO;
        float p_term     = drag_ff + g_SPEED_KP_NM * speed_error;
        int cuts_active  = (grip_factor < 0.99f) || (steer_factor < 0.99f);
        if (!cuts_active && p_term < DRIVER_TORQUE_NM) { // anti-windup: only integrate when throttle is free to respond
            s_speed_integral += g_SPEED_KI_NM * speed_error * CONTROL_DT_S;
            if (s_speed_integral > g_SPEED_I_MAX_NM) s_speed_integral = g_SPEED_I_MAX_NM;
            if (s_speed_integral < 0.0f) s_speed_integral = 0.0f;
        }

        driver_torque = p_term + s_speed_integral;

        driver_torque *= grip_factor; // apply both cuts to the full demand
        driver_torque *= steer_factor;

        if (driver_torque < 0.0f) driver_torque = 0.0f;
        if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;
    } else {
        s_speed_integral = 0.0f; // braking is pure-P, reset the throttle integrator

        driver_torque = g_BRAKE_KP_NM * speed_error;
        if (driver_torque < DRIVER_BRAKE_NM) driver_torque = DRIVER_BRAKE_NM;
        if (driver_torque > 0.0f) driver_torque = 0.0f;
    }

    return driver_torque;
}
