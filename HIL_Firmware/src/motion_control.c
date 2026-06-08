#include "../include/motion_control.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/* ---- Helpers ---- */

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

static float nearest_cone_dist(float x, float y,
                                const TrackPoint *cones, int n)
{
    float min_d2 = 1e9f;
    int i;
    for (i = 0; i < n; i++) {
        float dx = cones[i].x - x;
        float dy = cones[i].y - y;
        float d2 = dx*dx + dy*dy;
        if (d2 < min_d2) min_d2 = d2;
    }
    return sqrtf(min_d2);
}

static float wrap_pi(float a)
{
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

/* ---- Two-pass speed planner ---- */

/*
 * Set a speed cap per upcoming waypoint from curvature, then back-propagate
 * braking limits. start_idx is the controller's progress index.
 */
static float plan_target_speed(const VehicleState *state, const Track *track,
                               int start_idx)
{
    float v_limit[SPEED_PLAN_STEPS];
    float seg_len[SPEED_PLAN_STEPS];  /* distance from point i to i+1 */
    int   count = track->count;
    int   n     = 0;
    float path_s = 0.0f;
    int   i;

    /* Forward pass: speed cap from curvature at each upcoming waypoint. */
    for (i = 0; i < SPEED_PLAN_STEPS; i++) {
        int ccur  = (start_idx + i    ) % count;

        /* Curvature here, taken as the max over several stencil spacings so a
         * sharp apex is not under-read. */
        float kappa = 0.0f;
        for (int d = 1; d <= 3; d++) {
            int cprev = (start_idx + i - d + count) % count;
            int cnext = (start_idx + i + d        ) % count;
            float k = menger_curvature(
                track->points[cprev].x, track->points[cprev].y,
                track->points[ccur].x,  track->points[ccur].y,
                track->points[cnext].x, track->points[cnext].y);
            if (k > kappa) kappa = k;
        }
        int cnext = (start_idx + i + 1) % count;

        float v_curve = (kappa > 1e-4f)
                        ? sqrtf(MAX_LATERAL_ACCEL_MS2 / kappa)
                        : TARGET_SPEED_MS;
        v_limit[n] = (v_curve < TARGET_SPEED_MS) ? v_curve : TARGET_SPEED_MS;

        float dx = track->points[cnext].x - track->points[ccur].x;
        float dy = track->points[cnext].y - track->points[ccur].y;
        seg_len[n] = sqrtf(dx*dx + dy*dy);
        n++;

        path_s += seg_len[n - 1];
        if (path_s > SPEED_PLAN_HORIZON_M) break;
    }

    if (n == 0) return TARGET_SPEED_MS;

    /* Backward pass: propagate braking from furthest waypoint. */
    float v_sweep = v_limit[n - 1];
    for (i = n - 2; i >= 0; i--) {
        float v_can_arrive = sqrtf(v_sweep*v_sweep
                                   + 2.0f * MAX_BRAKE_DECEL_MS2 * seg_len[i]);
        v_sweep = (v_limit[i] < v_can_arrive) ? v_limit[i] : v_can_arrive;
    }

    /* Final step: braking from the car to the first upcoming waypoint. */
    {
        int   wp0   = start_idx % count;
        float ddx   = track->points[wp0].x - state->x;
        float ddy   = track->points[wp0].y - state->y;
        float d_now = sqrtf(ddx*ddx + ddy*ddy);
        float v_now = sqrtf(v_sweep*v_sweep + 2.0f * MAX_BRAKE_DECEL_MS2 * d_now);
        if (v_now < TARGET_SPEED_MS) return v_now;
    }

    return TARGET_SPEED_MS;
}


/* ---- Nearest-segment finder ---- */

/*
 * Find the racing-line segment nearest to (px, py) near center_idx. Returns the
 * segment's start index, writes the clamped projection t in [0,1] to *out_t and
 * the signed cross-track error (+ve when right of the path) to *out_cte.
 */
static int find_nearest_segment(const Track *track, float px, float py,
                                int center_idx, float *out_t, float *out_cte)
{
    int   count    = track->count;
    int   best     = center_idx;
    float best_d2  = 1e18f;
    float best_t   = 0.0f;
    float best_cte = 0.0f;
    int   k;

    for (k = -NEAREST_SEARCH_BACK; k <= NEAREST_SEARCH_FWD; k++) {
        int   i0 = ((center_idx + k) % count + count) % count;
        int   i1 = (i0 + 1) % count;

        float ax = track->points[i0].x, ay = track->points[i0].y;
        float ex = track->points[i1].x - ax;
        float ey = track->points[i1].y - ay;
        float len2 = ex*ex + ey*ey;
        if (len2 < 1e-6f) continue;

        float t = ((px - ax) * ex + (py - ay) * ey) / len2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float cx = ax + t * ex;
        float cy = ay + t * ey;
        float dx = px - cx, dy = py - cy;
        float d2 = dx*dx + dy*dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            best    = i0;
            best_t  = t;
            /* signed CTE: +ve when right of the segment direction */
            float inv = 1.0f / sqrtf(len2);
            best_cte  = (dx * (-ey) + dy * ex) * inv * -1.0f;
        }
    }

    *out_t = best_t;
    if (out_cte) *out_cte = best_cte;
    return best;
}


/*
 * Max curvature (1/m) over the waypoints within `span` metres ahead of
 * start_idx. Uses the same multi-spacing stencil as the speed planner.
 */
static float lookahead_curvature(const Track *track, int start_idx, float span)
{
    int   count = track->count;
    float acc   = 0.0f;
    float kmax  = 0.0f;
    int   i;

    for (i = 0; i < count; i++) {
        int ccur = (start_idx + i) % count;
        for (int d = 1; d <= 3; d++) {
            int cprev = (start_idx + i - d + count) % count;
            int cnext = (start_idx + i + d        ) % count;
            float k = menger_curvature(
                track->points[cprev].x, track->points[cprev].y,
                track->points[ccur].x,  track->points[ccur].y,
                track->points[cnext].x, track->points[cnext].y);
            if (k > kmax) kmax = k;
        }
        int nxt = (start_idx + i + 1) % count;
        float dx = track->points[nxt].x - track->points[ccur].x;
        float dy = track->points[nxt].y - track->points[ccur].y;
        acc += sqrtf(dx*dx + dy*dy);
        if (acc > span) break;
    }
    return kmax;
}


/* ---- Pure Pursuit look-ahead point finder ---- */

/*
 * Walk the racing line forward from start_idx until Ld metres of arc length,
 * then write that point to (*lx,*ly). Stays on the line wherever the car is.
 */
static void lookahead_point(const Track *track, int start_idx,
                            float Ld, float *lx, float *ly)
{
    int   count = track->count;
    float acc   = 0.0f;
    int   i;

    for (i = 0; i < count; i++) {
        int a = (start_idx + i)     % count;
        int b = (start_idx + i + 1) % count;
        float ex = track->points[b].x - track->points[a].x;
        float ey = track->points[b].y - track->points[a].y;
        float seg = sqrtf(ex*ex + ey*ey);
        if (acc + seg >= Ld) {
            float r = (seg > 1e-6f) ? (Ld - acc) / seg : 0.0f;
            *lx = track->points[a].x + r * ex;
            *ly = track->points[a].y + r * ey;
            return;
        }
        acc += seg;
    }
    /* Fallback: aim at the start waypoint. */
    *lx = track->points[start_idx % count].x;
    *ly = track->points[start_idx % count].y;
}


/* ---- Boundary steering correction (safety net) ---- */

/*
 * Signed steering nudge away from whichever cone boundary the car nears:
 * positive steers away from left cones, negative away from right cones.
 */
static float boundary_steer_correction(float x, float y, const Track *track)
{
    float dl = nearest_cone_dist(x, y, track->left_cones,  track->left_count);
    float dr = nearest_cone_dist(x, y, track->right_cones, track->right_count);
    float corr = 0.0f;

    if (dl < BOUNDARY_WARN_M)
        corr += (1.0f - dl / BOUNDARY_WARN_M) * BOUNDARY_CORR_GAIN;
    if (dr < BOUNDARY_WARN_M)
        corr -= (1.0f - dr / BOUNDARY_WARN_M) * BOUNDARY_CORR_GAIN;

    if (corr >  BOUNDARY_CORR_GAIN) corr =  BOUNDARY_CORR_GAIN;
    if (corr < -BOUNDARY_CORR_GAIN) corr = -BOUNDARY_CORR_GAIN;
    return corr;
}


/* ---- Public update ---- */

float motion_control_update(VehicleState *state, const Track *track,
                            float *out_target_speed)
{
    /* Own progress index along the line, independent of track->current_index
     * (which can jump ahead when the car slides wide and skip a corner). */
    static int   s_path_idx       = -1;
    static float s_speed_integral = 0.0f;   /* throttle integral state, Nm */

    int   count = track->count;
    float vx    = state->velocity;

    if (s_path_idx < 0 || s_path_idx >= count) s_path_idx = track->current_index;

    /* STEERING: Pure Pursuit (geometric look-ahead) + cone safety net. */

    /* Pure Pursuit steers the rear axle through a look-ahead point. The front
     * axle is projected to advance the progress index and read the CTE trim. */
    float ra_x = state->x - CG_TO_REAR_M * cosf(state->heading);
    float ra_y = state->y - CG_TO_REAR_M * sinf(state->heading);
    float fa_x = state->x + CG_TO_FRONT_M * cosf(state->heading);
    float fa_y = state->y + CG_TO_FRONT_M * sinf(state->heading);

    float t, cte;
    int   i0 = find_nearest_segment(track, fa_x, fa_y, s_path_idx, &t, &cte);

    s_path_idx = i0;

    /* Speed-adaptive look-ahead: short in slow corners, long at speed. */
    float Ld = K_LOOKAHEAD * vx;
    if (Ld < LOOKAHEAD_MIN_M) Ld = LOOKAHEAD_MIN_M;
    if (Ld > LOOKAHEAD_MAX_M) Ld = LOOKAHEAD_MAX_M;

    /* Curvature-aware look-ahead floor: tie the floor to corner radius so the
     * look-ahead point lands near the apex in tight corners. Only lowers Ld. */
    {
        float kappa_ahead = lookahead_curvature(track, i0, Ld);
        if (kappa_ahead > 1e-4f) {
            float ld_floor = K_LD_RADIUS / kappa_ahead;   /* = K_LD_RADIUS * R */
            if (ld_floor < LOOKAHEAD_ABS_MIN) ld_floor = LOOKAHEAD_ABS_MIN;
            if (ld_floor < LOOKAHEAD_MIN_M && Ld > ld_floor) Ld = ld_floor;
        }
    }

    float lx, ly;
    lookahead_point(track, i0, Ld, &lx, &ly);

    /* alpha = angle from car heading to the look-ahead point at the rear axle. */
    float dx_l    = lx - ra_x;
    float dy_l    = ly - ra_y;
    float Ld_act  = sqrtf(dx_l*dx_l + dy_l*dy_l);
    if (Ld_act < 1e-3f) Ld_act = 1e-3f;
    float alpha   = wrap_pi(atan2f(dy_l, dx_l) - state->heading);
    float steer_pp = atan2f(2.0f * WHEELBASE_M * sinf(alpha), Ld_act);

    /* Cross-track restoring trim: small proportional pull back onto the line.
     * cte > 0 (right of line) steers left (+). */
    float steer = steer_pp + K_CTE_PP * cte;

    /* Gentle cone repulsion as a safety net. */
    steer += boundary_steer_correction(state->x, state->y, track);

    if (steer >  MAX_STEER_RAD) steer =  MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;

    /* Slew-rate limit: cap how far the commanded angle moves in one tick. */
    {
        float max_step = MAX_STEER_RATE_RADS * CONTROL_DT_S;
        float dsteer   = steer - state->steering;
        if (dsteer >  max_step) steer = state->steering + max_step;
        if (dsteer < -max_step) steer = state->steering - max_step;
    }

    state->steering = steer;

    /* SPEED: two-pass planner + boundary proximity reduction. */

    float target_speed = plan_target_speed(state, track, s_path_idx);

    float dl = nearest_cone_dist(state->x, state->y,
                                 track->left_cones, track->left_count);
    float dr = nearest_cone_dist(state->x, state->y,
                                 track->right_cones, track->right_count);
    float min_d = (dl < dr) ? dl : dr;

    if (min_d < BOUNDARY_SLOW_M) {
        float blend       = min_d / BOUNDARY_SLOW_M;   /* 0 at cone, 1 at margin */
        float speed_floor = BOUNDARY_SLOW_FACTOR * target_speed;
        target_speed = speed_floor + blend * (target_speed - speed_floor);
    }

    /* TORQUE: P-throttle + drag feedforward; P-brake. */

    if (out_target_speed) *out_target_speed = target_speed;

    float speed_error = target_speed - vx;
    float driver_torque;

    if (speed_error >= 0.0f) {
        /* Traction circle: cut throttle by the lateral grip already in use, so
         * the car powers up only as the corner opens. Uses lagged ay_filt. */
        float lat_ratio = fabsf(state->ay_filt) / LAT_GRIP_REF_MS2;
        if (lat_ratio > 1.0f) lat_ratio = 1.0f;
        float grip_factor = sqrtf(1.0f - lat_ratio * lat_ratio);

        /* Steering-saturation cut: near full lock the front tyres spend their
         * grip turning, so fade throttle from STEER_SAT_FRAC of lock to zero. */
        float steer_factor = 1.0f;
        float steer_frac   = fabsf(state->steering) / MAX_STEER_RAD;
        if (steer_frac > STEER_SAT_FRAC) {
            float over = (steer_frac - STEER_SAT_FRAC) / (1.0f - STEER_SAT_FRAC);
            if (over > 1.0f) over = 1.0f;
            steer_factor = 1.0f - over;
        }

        /* Anti-windup: only advance the integrator when throttle is free to
         * respond (no cut active and P-term below the motor cap). */
        float p_term      = DRAG_FF_NM * vx + SPEED_KP_NM * speed_error;
        int   cuts_active = (grip_factor < 0.99f) || (steer_factor < 0.99f);
        if (!cuts_active && p_term < DRIVER_TORQUE_NM) {
            s_speed_integral += SPEED_KI_NM * speed_error * CONTROL_DT_S;
            if (s_speed_integral >  SPEED_I_MAX_NM) s_speed_integral =  SPEED_I_MAX_NM;
            if (s_speed_integral <  0.0f)           s_speed_integral =  0.0f;
        }

        driver_torque = p_term + s_speed_integral;

        /* Apply both cuts to the full demand (P + I). */
        driver_torque *= grip_factor;
        driver_torque *= steer_factor;

        if (driver_torque < 0.0f)             driver_torque = 0.0f;
        if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;
    } else {
        /* Braking is pure-P. Reset the throttle integrator. */
        s_speed_integral = 0.0f;

        driver_torque = BRAKE_KP_NM * speed_error;
        if (driver_torque < DRIVER_BRAKE_NM) driver_torque = DRIVER_BRAKE_NM;
        if (driver_torque > 0.0f)            driver_torque = 0.0f;
    }

    return driver_torque;
}
