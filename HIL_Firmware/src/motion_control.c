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
 * Build v_limit[i] for each of the next SPEED_PLAN_STEPS waypoints from
 * curvature, then back-propagate braking constraints from furthest to
 * nearest.  A final step applies the braking distance from the car's current
 * position to the first upcoming waypoint.
 *
 * start_idx is the controller's progress index (the segment the car is
 * currently on), NOT track->current_index.
 */
static float plan_target_speed(const VehicleState *state, const Track *track,
                               int start_idx)
{
    float v_limit[SPEED_PLAN_STEPS];
    float seg_len[SPEED_PLAN_STEPS];  /* seg_len[i] = distance from point i to point i+1 */
    int   count = track->count;
    int   n     = 0;
    float path_s = 0.0f;
    int   i;

    /*
     * Forward pass.  v_limit[i] is the speed cap from the curvature AT the
     * (start_idx + i)-th waypoint — including start_idx itself, so the cap of
     * the corner the car is currently in is never skipped.
     */
    for (i = 0; i < SPEED_PLAN_STEPS; i++) {
        int ccur  = (start_idx + i    ) % count;

        /*
         * Curvature at this point, taken as the MAX over several stencil
         * spacings (±1, ±2, ±3 waypoints).  A single ±1 stencil on ~2.5 m
         * spacing under-reads a sharp hairpin apex (three closely spaced points
         * look almost straight), letting the planner carry too much speed into
         * the corner so the car understeers wide.  Sampling wider baselines and
         * keeping the worst case captures the true apex sharpness without
         * over-slowing gentle corners.
         */
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

    /* --- Backward pass: propagate braking from furthest waypoint --- */
    float v_sweep = v_limit[n - 1];
    for (i = n - 2; i >= 0; i--) {
        float v_can_arrive = sqrtf(v_sweep*v_sweep
                                   + 2.0f * MAX_BRAKE_DECEL_MS2 * seg_len[i]);
        v_sweep = (v_limit[i] < v_can_arrive) ? v_limit[i] : v_can_arrive;
    }

    /* --- Final step: from car position to the first upcoming waypoint --- */
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
 * Find the racing-line segment nearest to (px, py) within a window around
 * center_idx.  Returns the index of the segment's start waypoint and writes
 * the clamped projection parameter t in [0,1] to *out_t.  Also writes the
 * signed cross-track error (perpendicular distance, +ve when the point is to
 * the RIGHT of the path direction) to *out_cte if non-NULL.
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
            /* signed CTE: +ve when (px,py) is right of the segment direction */
            float inv = 1.0f / sqrtf(len2);
            best_cte  = (dx * (-ey) + dy * ex) * inv * -1.0f;
        }
    }

    *out_t = best_t;
    if (out_cte) *out_cte = best_cte;
    return best;
}


/* ---- Pure Pursuit look-ahead point finder ---- */

/*
 * Walk the racing line forward from (start_idx) accumulating arc length until at
 * least Ld metres past the car's projection, then return the point that far
 * ahead.  Writes the look-ahead point to (*lx,*ly).  Uses the path geometry
 * directly so the target sits on the line regardless of how far the car has
 * been pushed off it.
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
    /* Fallback (degenerate track): aim at the start waypoint. */
    *lx = track->points[start_idx % count].x;
    *ly = track->points[start_idx % count].y;
}


/* ---- Boundary steering correction (safety net) ---- */

/*
 * Returns a signed steering correction that nudges the car away from whichever
 * cone boundary it is approaching:
 *   positive  -> steer toward +heading-normal (away from left/blue cones)
 *   negative  -> steer the other way (away from right/yellow cones)
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
    /*
     * The controller tracks its own progress index along the racing line with
     * continuity, independent of track->current_index (which can jump ahead
     * when the car slides wide and would otherwise make the planner skip a
     * corner).  Persists between ticks; the simulation drives one car.
     */
    static int s_path_idx = -1;

    int   count = track->count;
    float vx    = state->velocity;

    if (s_path_idx < 0 || s_path_idx >= count) s_path_idx = track->current_index;

    /* ------------------------------------------------------------------ */
    /* STEERING — Pure Pursuit (geometric look-ahead) + cone safety net     */
    /* ------------------------------------------------------------------ */

    /*
     * Pure Pursuit references the REAR axle: it finds a target point on the
     * racing line a look-ahead distance ahead and computes the single-arc steer
     * that drives the rear axle through it.  We still project the FRONT axle to
     * advance the progress index and to read the signed cross-track error for a
     * small restoring trim.
     */
    float ra_x = state->x - CG_TO_REAR_M * cosf(state->heading);
    float ra_y = state->y - CG_TO_REAR_M * sinf(state->heading);
    float fa_x = state->x + CG_TO_FRONT_M * cosf(state->heading);
    float fa_y = state->y + CG_TO_FRONT_M * sinf(state->heading);

    float t, cte;
    int   i0 = find_nearest_segment(track, fa_x, fa_y, s_path_idx, &t, &cte);

    /* Advance the controller's progress index with continuity */
    s_path_idx = i0;

    /* Speed-adaptive look-ahead: short in slow corners (commit to the apex),
     * long at speed (smooth, stable line). */
    float Ld = K_LOOKAHEAD * vx;
    if (Ld < LOOKAHEAD_MIN_M) Ld = LOOKAHEAD_MIN_M;
    if (Ld > LOOKAHEAD_MAX_M) Ld = LOOKAHEAD_MAX_M;

    float lx, ly;
    lookahead_point(track, i0, Ld, &lx, &ly);

    /* alpha = angle from the car heading to the look-ahead point, measured at
     * the rear axle.  delta = atan2(2 L sin(alpha), Ld). */
    float dx_l    = lx - ra_x;
    float dy_l    = ly - ra_y;
    float Ld_act  = sqrtf(dx_l*dx_l + dy_l*dy_l);
    if (Ld_act < 1e-3f) Ld_act = 1e-3f;
    float alpha   = wrap_pi(atan2f(dy_l, dx_l) - state->heading);
    float steer_pp = atan2f(2.0f * WHEELBASE_M * sinf(alpha), Ld_act);

    /* Cross-track restoring trim: Pure Pursuit alone has no term that pulls the
     * car back onto the line once it and the look-ahead point share an offset;
     * add a small proportional pull.  cte > 0 => car is right of the line =>
     * steer left (+). */
    float steer = steer_pp + K_CTE_PP * cte;

    /* Gentle cone repulsion as a safety net */
    steer += boundary_steer_correction(state->x, state->y, track);

    if (steer >  MAX_STEER_RAD) steer =  MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;

    /* Slew-rate limit: a driver / steering actuator cannot snap the wheel
     * instantly, so cap how far the commanded angle may move in one tick.
     * state->steering still holds last tick's command. */
    {
        float max_step = MAX_STEER_RATE_RADS * MC_DT_S;
        float dsteer   = steer - state->steering;
        if (dsteer >  max_step) steer = state->steering + max_step;
        if (dsteer < -max_step) steer = state->steering - max_step;
    }

    state->steering = steer;

    /* ------------------------------------------------------------------ */
    /* SPEED — two-pass planner + boundary proximity reduction              */
    /* ------------------------------------------------------------------ */

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

    /* ------------------------------------------------------------------ */
    /* TORQUE — P-throttle + drag feedforward; P-brake                     */
    /* ------------------------------------------------------------------ */

    if (out_target_speed) *out_target_speed = target_speed;

    float speed_error = target_speed - vx;
    float driver_torque;

    if (speed_error >= 0.0f) {
        driver_torque = DRAG_FF_NM * vx + SPEED_KP_NM * speed_error;

        /* Traction circle: cut throttle in proportion to lateral grip already
         * in use, so the car only powers up as the corner opens (ay -> 0).
         * Use the roll-lagged ay_filt, not the raw same-tick ay: the latter
         * carries single-tick noise that would chop the throttle 0<->max. */
        float lat_ratio = fabsf(state->ay_filt) / LAT_GRIP_REF_MS2;
        if (lat_ratio > 1.0f) lat_ratio = 1.0f;
        float grip_factor = sqrtf(1.0f - lat_ratio * lat_ratio);
        driver_torque *= grip_factor;

        /* Steering-saturation cut: when the wheel is near full lock the front
         * tyres are already spending their whole grip budget turning, so any
         * throttle just powers the car wide (this was the mid-hairpin "pinned
         * at full lock and accelerating" failure of the old controller).  Fade
         * throttle from full at STEER_SAT_FRAC of the limit to zero at the
         * limit. */
        float steer_frac = fabsf(state->steering) / MAX_STEER_RAD;
        if (steer_frac > STEER_SAT_FRAC) {
            float over = (steer_frac - STEER_SAT_FRAC) / (1.0f - STEER_SAT_FRAC);
            if (over > 1.0f) over = 1.0f;
            driver_torque *= (1.0f - over);
        }

        if (driver_torque < 0.0f)             driver_torque = 0.0f;
        if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;
    } else {
        driver_torque = BRAKE_KP_NM * speed_error;
        if (driver_torque < DRIVER_BRAKE_NM) driver_torque = DRIVER_BRAKE_NM;
        if (driver_torque > 0.0f)            driver_torque = 0.0f;
    }

    return driver_torque;
}
