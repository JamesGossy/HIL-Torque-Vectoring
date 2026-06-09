#include "../include/motion_control.h"
#include "../include/lqr_steer.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/* ---- Helpers ---- */

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

static float nearest_cone_dist(float x, float y, const TrackPoint *cones, int n)
{
    float min_d2 = 1e9f;
    int i;
    for (i = 0; i < n; i++) {
        float dx = cones[i].x - x;
        float dy = cones[i].y - y;
        float d2 = dx * dx + dy * dy;
        if (d2 < min_d2) min_d2 = d2;
    }
    return sqrtf(min_d2);
}

static float wrap_pi(float a)
{
    while (a > PI)
        a -= 2.0f * PI;
    while (a < -PI)
        a += 2.0f * PI;
    return a;
}

/* ---- Two-pass speed planner ---- */

/*
 * Available braking deceleration, m/s^2, when the car is already using lateral
 * grip at speed v on a corner of curvature kappa. The tyre shares one
 * friction-circle budget GG_ACCEL_MS2 between cornering and braking:
 *
 *     a_lat^2 + a_lon^2 <= GG_ACCEL_MS2^2,   a_lat = v^2 * kappa
 *  => a_lon = sqrt(GG^2 - a_lat^2)
 *
 * so the harder the car is cornering, the less is left for braking. The result
 * is additionally capped at MAX_BRAKE_DECEL_MS2 (the powertrain/regen limit) and
 * floored at a small positive value so the backward sweep always makes progress.
 */
static float brake_decel_avail(float v, float kappa)
{
    /* Combined friction-circle budget grows with downforce (v^2), same as the
     * lateral grip: a_lat^2 + a_lon^2 <= gg(v)^2. So at speed (where braking
     * happens, before a corner) there is more total grip to brake with. */
    float gg_acc = lateral_grip_accel(GG_ACCEL_MS2, v);
    float a_lat  = v * v * kappa;
    float gg     = gg_acc * gg_acc;
    float a_lat2 = a_lat * a_lat;
    float a_lon  = (a_lat2 < gg) ? sqrtf(gg - a_lat2) : 0.0f;
    if (a_lon > MAX_BRAKE_DECEL_MS2) a_lon = MAX_BRAKE_DECEL_MS2;
    if (a_lon < 0.5f) a_lon = 0.5f; /* keep the sweep progressing */
    return a_lon;
}

/*
 * Set a speed cap per upcoming waypoint from curvature, then back-propagate
 * braking limits under the friction circle. start_idx is the controller's
 * progress index.
 */
static float plan_target_speed(const VehicleState *state, const Track *track, int start_idx)
{
    float v_limit[SPEED_PLAN_STEPS];
    float seg_len[SPEED_PLAN_STEPS]; /* distance from point i to i+1 */
    float seg_k[SPEED_PLAN_STEPS];   /* curvature at point i, for the GG budget */
    int count    = track->count;
    int n        = 0;
    float path_s = 0.0f;
    int i;

    /* Forward pass: speed cap from curvature at each upcoming waypoint. */
    for (i = 0; i < SPEED_PLAN_STEPS; i++) {
        int ccur = (start_idx + i) % count;

        /* Curvature here, taken as the max over several stencil spacings so a
         * sharp apex is not under-read. */
        float kappa = 0.0f;
        for (int d = 1; d <= 3; d++) {
            int cprev = (start_idx + i - d + count) % count;
            int cnext = (start_idx + i + d) % count;
            float k   = menger_curvature(track->points[cprev].x, track->points[cprev].y,
                track->points[ccur].x, track->points[ccur].y, track->points[cnext].x,
                track->points[cnext].y);
            if (k > kappa) kappa = k;
        }
        int cnext = (start_idx + i + 1) % count;

        /* Apex speed under speed-dependent grip: the conservative base budget
         * MAX_LATERAL_ACCEL_MS2 plus a FRACTION of the downforce bonus. Using the
         * full downforce grip here leaves the tracker no margin (it then washes
         * wide and saturates the steering recovering); PLANNER_DOWNFORCE_FRAC
         * keeps the on-car plan conservative while still gaining most of the
         * downforce in the faster corners. The LINE is shaped for the full grip;
         * only what the car DRIVES is held back. */
        float planner_base = MAX_LATERAL_ACCEL_MS2;
        float full         = apex_speed(planner_base, kappa, TARGET_SPEED_MS);
        float flat         = (kappa > 1e-4f) ? sqrtf(planner_base / kappa) : TARGET_SPEED_MS;
        if (flat > TARGET_SPEED_MS) flat = TARGET_SPEED_MS;
        v_limit[n] = flat + PLANNER_DOWNFORCE_FRAC * (full - flat);
        seg_k[n]   = kappa;

        float dx   = track->points[cnext].x - track->points[ccur].x;
        float dy   = track->points[cnext].y - track->points[ccur].y;
        seg_len[n] = sqrtf(dx * dx + dy * dy);
        n++;

        path_s += seg_len[n - 1];
        if (path_s > SPEED_PLAN_HORIZON_M) break;
    }

    if (n == 0) return TARGET_SPEED_MS;

    /* Backward pass: propagate braking from the furthest waypoint. The decel
     * available over segment i is set by the friction circle at the DOWNSTREAM
     * end (point i+1) - the deeper-into-the-corner end, where lateral load is
     * highest and braking room is smallest. As an apex approaches, that room
     * collapses, so the brake point is pushed back onto the preceding straight. */
    float v_sweep = v_limit[n - 1];
    for (i = n - 2; i >= 0; i--) {
        float a_brake      = brake_decel_avail(v_sweep, seg_k[i + 1]);
        float v_can_arrive = sqrtf(v_sweep * v_sweep + 2.0f * a_brake * seg_len[i]);
        v_sweep            = (v_limit[i] < v_can_arrive) ? v_limit[i] : v_can_arrive;
    }

    /* Final step: braking from the car to the first upcoming waypoint. */
    {
        int wp0       = start_idx % count;
        float ddx     = track->points[wp0].x - state->x;
        float ddy     = track->points[wp0].y - state->y;
        float d_now   = sqrtf(ddx * ddx + ddy * ddy);
        float a_brake = brake_decel_avail(v_sweep, seg_k[0]);
        float v_now   = sqrtf(v_sweep * v_sweep + 2.0f * a_brake * d_now);
        if (v_now < TARGET_SPEED_MS) return v_now;
    }

    return TARGET_SPEED_MS;
}


/* ---- Nearest-segment finder ---- */

/*
 * Find the racing-line segment nearest to (px, py) near center_idx. Returns the
 * segment's start index and writes the signed cross-track error (+ve when right
 * of the path) to *out_cte.
 */
static int find_nearest_segment(
    const Track *track, float px, float py, int center_idx, float *out_cte)
{
    int count      = track->count;
    int best       = center_idx;
    float best_d2  = 1e18f;
    float best_cte = 0.0f;
    int k;

    for (k = -NEAREST_SEARCH_BACK; k <= NEAREST_SEARCH_FWD; k++) {
        int i0 = ((center_idx + k) % count + count) % count;
        int i1 = (i0 + 1) % count;

        float ax = track->points[i0].x, ay = track->points[i0].y;
        float ex   = track->points[i1].x - ax;
        float ey   = track->points[i1].y - ay;
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
            /* signed CTE: +ve when right of the segment direction */
            float inv = 1.0f / sqrtf(len2);
            best_cte  = (dx * (-ey) + dy * ex) * inv * -1.0f;
        }
    }

    if (out_cte) *out_cte = best_cte;
    return best;
}


/*
 * Max curvature (1/m) over the waypoints within `span` metres ahead of
 * start_idx. Uses the same multi-spacing stencil as the speed planner.
 */
static float lookahead_curvature(const Track *track, int start_idx, float span)
{
    int count  = track->count;
    float acc  = 0.0f;
    float kmax = 0.0f;
    int i;

    for (i = 0; i < count; i++) {
        int ccur = (start_idx + i) % count;
        for (int d = 1; d <= 3; d++) {
            int cprev = (start_idx + i - d + count) % count;
            int cnext = (start_idx + i + d) % count;
            float k   = menger_curvature(track->points[cprev].x, track->points[cprev].y,
                track->points[ccur].x, track->points[ccur].y, track->points[cnext].x,
                track->points[cnext].y);
            if (k > kmax) kmax = k;
        }
        int nxt  = (start_idx + i + 1) % count;
        float dx = track->points[nxt].x - track->points[ccur].x;
        float dy = track->points[nxt].y - track->points[ccur].y;
        acc += sqrtf(dx * dx + dy * dy);
        if (acc > span) break;
    }
    return kmax;
}


/* ---- Boundary steering correction (safety net) ---- */

/*
 * Signed steering nudge away from whichever cone boundary the car nears:
 * positive steers away from left cones, negative away from right cones.
 */
static float boundary_steer_correction(float x, float y, const Track *track)
{
    float dl   = nearest_cone_dist(x, y, track->left_cones, track->left_count);
    float dr   = nearest_cone_dist(x, y, track->right_cones, track->right_count);
    float corr = 0.0f;

    if (dl < BOUNDARY_WARN_M) corr += (1.0f - dl / BOUNDARY_WARN_M) * BOUNDARY_CORR_GAIN;
    if (dr < BOUNDARY_WARN_M) corr -= (1.0f - dr / BOUNDARY_WARN_M) * BOUNDARY_CORR_GAIN;

    if (corr > BOUNDARY_CORR_GAIN) corr = BOUNDARY_CORR_GAIN;
    if (corr < -BOUNDARY_CORR_GAIN) corr = -BOUNDARY_CORR_GAIN;
    return corr;
}


/* ---- Controller memory ---- */

/* Progress index along the line, independent of track->current_index (which can
 * jump ahead when the car slides wide and skips a corner), and the throttle
 * integrator. File-scope so motion_control_reset() can clear them between runs.
 * s_path_idx = -1 means "uninitialised" - the first update seeds it. */
static int s_path_idx         = -1;
static float s_speed_integral = 0.0f; /* throttle integral state, Nm */

/* Reset the driver's internal state and the LQR steering state it owns. Call
 * before an independent run so no state leaks in from the previous one; the sim
 * entry points do this after track_init(). */
void motion_control_reset(void)
{
    s_path_idx       = -1;
    s_speed_integral = 0.0f;
    lqr_steer_reset();
}


/* ---- Public update ---- */

float motion_control_update(VehicleState *state, const Track *track, float *out_target_speed)
{
    int count = track->count;
    float vx  = state->velocity;

    if (s_path_idx < 0 || s_path_idx >= count) s_path_idx = track->current_index;

    /* STEERING: model-based LQR on the dynamic-bicycle error dynamics, plus a
     * cone-repulsion safety net. The front axle is projected onto the racing
     * line to read the cross-track error and advance the progress index. */
    float fa_x = state->x + CG_TO_FRONT_M * cosf(state->heading);
    float fa_y = state->y + CG_TO_FRONT_M * sinf(state->heading);

    float cte;
    int i0 = find_nearest_segment(track, fa_x, fa_y, s_path_idx, &cte);

    s_path_idx = i0;

    /* LQR error state: e1 = signed cross-track error, e2 = heading error vs the
     * path tangent at the nearest segment, path_kappa = local (signed) curvature. */
    float steer;
    {
        int i1   = (i0 + 1) % count;
        float ph = atan2f(
            track->points[i1].y - track->points[i0].y, track->points[i1].x - track->points[i0].x);
        float e2 = wrap_pi(state->heading - ph);
        /* e1 sign: cte is +ve when the car is right of the path; the error model
         * takes +e1 to the left, so negate. */
        float e1 = -cte;
        float kp = lookahead_curvature(track, i0, 1.0f); /* curvature at car */
        /* sign of curvature: + when the path turns left (CCW). Derive from the
         * change in path heading across the nearest segment. */
        int i2    = (i0 + 2) % count;
        float ph2 = atan2f(
            track->points[i2].y - track->points[i1].y, track->points[i2].x - track->points[i1].x);
        float dpsi         = wrap_pi(ph2 - ph);
        float kappa_signed = (dpsi >= 0.0f ? kp : -kp);

        steer = lqr_steer_command(vx, state->vy, e1, e2, state->yaw_rate, kappa_signed);
    }

    /* Gentle cone repulsion as a safety net. */
    steer += boundary_steer_correction(state->x, state->y, track);

    if (steer > MAX_STEER_RAD) steer = MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;

    /* Slew-rate limit: cap how far the commanded angle moves in one tick. */
    {
        float max_step = MAX_STEER_RATE_RADS * CONTROL_DT_S;
        float dsteer   = steer - state->steering;
        if (dsteer > max_step) steer = state->steering + max_step;
        if (dsteer < -max_step) steer = state->steering - max_step;
    }

    state->steering = steer;

    /* SPEED: two-pass planner + boundary proximity reduction. */

    float target_speed = plan_target_speed(state, track, s_path_idx);

    float dl    = nearest_cone_dist(state->x, state->y, track->left_cones, track->left_count);
    float dr    = nearest_cone_dist(state->x, state->y, track->right_cones, track->right_count);
    float min_d = (dl < dr) ? dl : dr;

    if (min_d < BOUNDARY_SLOW_M) {
        float blend       = min_d / BOUNDARY_SLOW_M; /* 0 at cone, 1 at margin */
        float speed_floor = BOUNDARY_SLOW_FACTOR * target_speed;
        target_speed      = speed_floor + blend * (target_speed - speed_floor);
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
        float p_term    = DRAG_FF_NM * vx + SPEED_KP_NM * speed_error;
        int cuts_active = (grip_factor < 0.99f) || (steer_factor < 0.99f);
        if (!cuts_active && p_term < DRIVER_TORQUE_NM) {
            s_speed_integral += SPEED_KI_NM * speed_error * CONTROL_DT_S;
            if (s_speed_integral > SPEED_I_MAX_NM) s_speed_integral = SPEED_I_MAX_NM;
            if (s_speed_integral < 0.0f) s_speed_integral = 0.0f;
        }

        driver_torque = p_term + s_speed_integral;

        /* Apply both cuts to the full demand (P + I). */
        driver_torque *= grip_factor;
        driver_torque *= steer_factor;

        if (driver_torque < 0.0f) driver_torque = 0.0f;
        if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;
    } else {
        /* Braking is pure-P. Reset the throttle integrator. */
        s_speed_integral = 0.0f;

        driver_torque = BRAKE_KP_NM * speed_error;
        if (driver_torque < DRIVER_BRAKE_NM) driver_torque = DRIVER_BRAKE_NM;
        if (driver_torque > 0.0f) driver_torque = 0.0f;
    }

    return driver_torque;
}
