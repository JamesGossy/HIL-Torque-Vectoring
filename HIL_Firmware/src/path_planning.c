#include "../include/path_planning.h"
#include "vehicle_config.h"
#include "grip_model.h"
#include "tunables.h"
#include <math.h>

/* Builds the racing line: detect gates, resample the centreline, then bend it
 * within the corridor to minimise lap time. */

static float left_x[MAX_CONES], left_y[MAX_CONES];
static float right_x[MAX_CONES], right_y[MAX_CONES];


#define PP_MAX_GATES     400
#define MAX_GATE_WIDTH_M 10.0f /* reject pairs wider than this */

typedef struct {
    TrackPoint left;
    TrackPoint right;
} Gate;

static Gate pp_gates[PP_MAX_GATES];
static int pp_n_gates;

/* Pair each left cone with its nearest right cone, dropping pairs that are too wide. */
static void extract_gates(int n_left, int n_right)
{
    int i, j;
    int nearest_right[MAX_CONES];
    float dx, dy, d, best_d;
    float max_w2 = MAX_GATE_WIDTH_M * MAX_GATE_WIDTH_M;

    /* nearest right cone for each left cone */
    for (i = 0; i < n_left; i++) {
        nearest_right[i] = 0;
        best_d           = 1e18f;
        for (j = 0; j < n_right; j++) {
            dx = right_x[j] - left_x[i];
            dy = right_y[j] - left_y[i];
            d  = dx * dx + dy * dy;
            if (d < best_d) {
                best_d           = d;
                nearest_right[i] = j;
            }
        }
    }

    /* keep pairs that are not too wide */
    pp_n_gates = 0;
    for (i = 0; i < n_left && pp_n_gates < PP_MAX_GATES; i++) { /* cones already in track order */
        int ri = nearest_right[i];
        dx     = right_x[ri] - left_x[i];
        dy     = right_y[ri] - left_y[i];
        if (dx * dx + dy * dy > max_w2) continue;

        pp_gates[pp_n_gates].left.x  = left_x[i];
        pp_gates[pp_n_gates].left.y  = left_y[i];
        pp_gates[pp_n_gates].right.x = right_x[ri];
        pp_gates[pp_n_gates].right.y = right_y[ri];
        pp_n_gates++;
    }
}


#define RESAMPLE_SPACING_M 2.5f /* target waypoint spacing, m */
#define OPT_PASSES 400

static float cl_x[PP_MAX_GATES];
static float cl_y[PP_MAX_GATES];
static float cl_h[PP_MAX_GATES]; /* corridor half-width */

static float rs_x[MAX_WAYPOINTS];
static float rs_y[MAX_WAYPOINTS];
static float rs_h[MAX_WAYPOINTS];   /* half-width */
static float rs_nx[MAX_WAYPOINTS];  /* unit track normal */
static float rs_ny[MAX_WAYPOINTS];
static float rs_off[MAX_WAYPOINTS]; /* lateral offset along the normal */
static int rs_n;

/* Build the centreline (gate midpoints and half-widths) from the gates. */
static void build_centreline(void)
{
    int i;
    for (i = 0; i < pp_n_gates; i++) {
        cl_x[i]  = 0.5f * (pp_gates[i].left.x + pp_gates[i].right.x);
        cl_y[i]  = 0.5f * (pp_gates[i].left.y + pp_gates[i].right.y);
        float dx = pp_gates[i].right.x - pp_gates[i].left.x;
        float dy = pp_gates[i].right.y - pp_gates[i].left.y;
        cl_h[i]  = 0.5f * sqrtf(dx * dx + dy * dy);
    }
}

/* Resample the closed centreline to uniform spacing, interpolating position and half-width. */
static void resample_centreline(void)
{
    int g = pp_n_gates;
    int i;
    float total = 0.0f;

    /* total centreline length */
    for (i = 0; i < g; i++) {
        int j = (i + 1) % g;
        total += sqrtf(
            (cl_x[j] - cl_x[i]) * (cl_x[j] - cl_x[i]) + (cl_y[j] - cl_y[i]) * (cl_y[j] - cl_y[i]));
    }

    /* pick point count and uniform step */
    int m = (int)(total / RESAMPLE_SPACING_M + 0.5f);
    if (m < 8) m = 8;
    if (m > MAX_WAYPOINTS) m = MAX_WAYPOINTS;
    float step = total / (float)m;

    int seg       = 0; /* O(m*g) re-walk from segment 0 per point, fine at track sizes */
    float seg_acc = 0.0f;
    float seg_len = 0.0f;
    (void)seg;
    (void)seg_acc;
    (void)seg_len;

    for (i = 0; i < m; i++) {
        float target = i * step;
        float walked = 0.0f;
        seg          = 0;
        seg_acc      = 0.0f;
        while (seg < g) {
            int j   = (seg + 1) % g;
            seg_len = sqrtf((cl_x[j] - cl_x[seg]) * (cl_x[j] - cl_x[seg])
                + (cl_y[j] - cl_y[seg]) * (cl_y[j] - cl_y[seg]));
            if (walked + seg_len >= target || seg == g - 1) {
                seg_acc = target - walked;
                break;
            }
            walked += seg_len;
            seg++;
        }
        int j   = (seg + 1) % g;
        float t = (seg_len > 1e-6f) ? (seg_acc / seg_len) : 0.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        rs_x[i] = cl_x[seg] + t * (cl_x[j] - cl_x[seg]);
        rs_y[i] = cl_y[seg] + t * (cl_y[j] - cl_y[seg]);
        rs_h[i] = cl_h[seg] + t * (cl_h[j] - cl_h[seg]);
    }
    rs_n = m;
}

/* Unit track normals from the resampled centreline. */
static void compute_normals(void)
{
    int i, n = rs_n;
    for (i = 0; i < n; i++) {
        int p     = (i - 1 + n) % n;
        int q     = (i + 1) % n;
        float tx  = rs_x[q] - rs_x[p];
        float ty  = rs_y[q] - rs_y[p];
        float len = sqrtf(tx * tx + ty * ty);
        if (len < 1e-6f) {
            rs_nx[i] = 0.0f;
            rs_ny[i] = 0.0f;
            continue;
        }
        rs_nx[i] = -ty / len; /* left-hand normal */
        rs_ny[i] = tx / len;
    }
}


/* ---- speed profile ---- */

/* Longitudinal accel/brake limits for the speed-profile passes, m/s^2. */
#ifndef PP_ACCEL_LON
#define PP_ACCEL_LON 6.0f
#endif
#ifndef PP_BRAKE_LON
#define PP_BRAKE_LON 9.0f
#endif
#define PP_V_MAX 30.0f /* top speed cap for the profile, m/s */

/* Feasibility floor radius, m: kinematic minimum (full lock) opened by
 * g_PP_RADIUS_FACTOR for the understeer/slip the kinematic model ignores. Both
 * inputs are tunable, so it is computed at runtime. */
static float pp_min_radius_m(void)
{
    float r_kin = WHEELBASE_M / tanf(g_MAX_STEER_RAD * ACK_NOMINAL);
    return g_PP_RADIUS_FACTOR * r_kin;
}
#define PP_CURV_PENALTY 50.0f /* fake lap-seconds per (1/m) over the floor, big enough to reject infeasible apexes */

#define CD_PASSES    60    /* anneal steps */
#define CD_DELTA0    0.60f /* initial offset perturbation, m */
#define CD_DELTA_MIN 0.02f /* stop annealing here, m */

static float rs_px(int i)
{
    return rs_x[i] + rs_off[i] * rs_nx[i];
}
static float rs_py(int i)
{
    return rs_y[i] + rs_off[i] * rs_ny[i];
}

/* Discrete curvature at point i of the current offset line, 1/m. */
static float line_curvature(int i, int n)
{
    int im1  = (i - 1 + n) % n;
    int ip1  = (i + 1) % n;
    float ax = rs_px(im1), ay = rs_py(im1);
    float bx = rs_px(i), by = rs_py(i);
    float cx = rs_px(ip1), cy = rs_py(ip1);
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

static float lt_ds[MAX_WAYPOINTS]; /* segment length i -> i+1, m */
static float lt_v[MAX_WAYPOINTS];  /* speed profile at point i, m/s */
static float lt_k[MAX_WAYPOINTS];  /* line curvature at point i, 1/m */

/* Longitudinal accel available at speed v on a segment of curvature k under the
 * friction circle, capped by the powertrain limit a_cap. */
static float lon_accel_avail(float v, float k, float a_cap)
{
    float gg     = peak_lat(v); /* line shaped for the full physical grip */
    float a_lat  = v * v * k;
    float budget = gg * gg - a_lat * a_lat;
    float a_lon  = (budget > 0.0f) ? sqrtf(budget) : 0.0f;
    if (a_lon > a_cap) a_lon = a_cap;
    if (a_lon < 0.5f) a_lon = 0.5f; /* floored so the sweep always progresses */
    return a_lon;
}

/* Lap time of the current offset line under a quasi-steady-state speed model. */
static float lap_time(int n)
{
    int i, lap;

    for (i = 0; i < n; i++) {
        int j    = (i + 1) % n;
        float dx = rs_px(j) - rs_px(i);
        float dy = rs_py(j) - rs_py(i);
        lt_ds[i] = sqrtf(dx * dx + dy * dy);

        float k = line_curvature(i, n);
        lt_k[i] = k;
        lt_v[i] = apex_speed(PEAK_LAT_FLAT, k, PP_V_MAX); /* downforce-aware apex cap */
    }

    /* Forward (traction) and backward (braking) passes under the friction
     * circle. Two laps so limits propagate past the wrap. The forward/backward
     * coupling is what rewards a late apex onto a straight. */
    for (lap = 0; lap < 2; lap++) {
        for (i = 0; i < n; i++) {
            int j       = (i + 1) % n;
            float alon  = lon_accel_avail(lt_v[i], lt_k[i], PP_ACCEL_LON);
            float reach = sqrtf(lt_v[i] * lt_v[i] + 2.0f * alon * lt_ds[i]);
            if (lt_v[j] > reach) lt_v[j] = reach;
        }
        for (i = n - 1; i >= 0; i--) {
            int j       = (i + 1) % n;
            float abrk  = lon_accel_avail(lt_v[j], lt_k[j], PP_BRAKE_LON);
            float reach = sqrtf(lt_v[j] * lt_v[j] + 2.0f * abrk * lt_ds[i]);
            if (lt_v[i] > reach) lt_v[i] = reach;
        }
    }

    float r_floor = pp_min_radius_m();
    float t       = 0.0f;
    for (i = 0; i < n; i++) {
        int j      = (i + 1) % n;
        float vavg = 0.5f * (lt_v[i] + lt_v[j]);
        if (vavg < 0.5f) vavg = 0.5f; /* guard near standstill */
        t += lt_ds[i] / vavg;

        float k       = lt_k[i];
        float r_hold  = r_floor; /* slow-corner (no downforce) floor */
        float a_dyn   = peak_lat(lt_v[i]);
        float r_speed = (a_dyn > 1e-3f) ? lt_v[i] * lt_v[i] / a_dyn : r_floor;
        if (r_speed < r_hold) r_hold = r_speed; /* tighter allowed where downforce holds it */
        float k_floor = 1.0f / r_hold;
        if (k > k_floor) t += PP_CURV_PENALTY * (k - k_floor);
    }
    return t;
}

/* ---- racing line optimisation ---- */

/* Seed the offsets with the minimum-curvature line so the lap-time search starts smooth. */
static void seed_min_curvature(int n)
{
    int pass, i;
    for (i = 0; i < n; i++)
        rs_off[i] = 0.0f;

    for (pass = 0; pass < OPT_PASSES; pass++) {
        int start = (pass % 2 == 0) ? 0 : n - 1;
        int end   = (pass % 2 == 0) ? n : -1;
        int step  = (pass % 2 == 0) ? 1 : -1;
        for (i = start; i != end; i += step) {
            int im2 = (i - 2 + n) % n, im1 = (i - 1 + n) % n;
            int ip1 = (i + 1) % n, ip2 = (i + 2) % n;
            float Sx
                = rs_px(im2) - 4.0f * rs_px(im1) + 6.0f * rs_x[i] - 4.0f * rs_px(ip1) + rs_px(ip2);
            float Sy
                = rs_py(im2) - 4.0f * rs_py(im1) + 6.0f * rs_y[i] - 4.0f * rs_py(ip1) + rs_py(ip2);
            float o   = -(rs_nx[i] * Sx + rs_ny[i] * Sy) / 6.0f;
            float lim = (1.0f - 2.0f * g_RACING_MARGIN) * rs_h[i];
            if (o > lim) o = lim;
            if (o < -lim) o = -lim;
            rs_off[i] = o;
        }
    }
}

/* Coordinate descent on the lap-time objective, refining the seeded line. */
static void optimize_racing_line(void)
{
    int pass, i, n = rs_n;

    for (i = 0; i < n; i++)
        rs_off[i] = 0.0f;
    if (n < 5) return;

    seed_min_curvature(n);

    float delta = CD_DELTA0;
    float best  = lap_time(n);

    for (pass = 0; pass < CD_PASSES && delta >= CD_DELTA_MIN; pass++) {
        int improved = 0;
        for (i = 0; i < n; i++) {
            float lim = (1.0f - 2.0f * g_RACING_MARGIN) * rs_h[i];
            float o0  = rs_off[i];

            for (int s = 0; s < 2; s++) { /* try + then - */
                float cand = o0 + (s == 0 ? delta : -delta);
                if (cand > lim) cand = lim;
                if (cand < -lim) cand = -lim;
                if (cand == rs_off[i]) continue;

                rs_off[i] = cand;
                float t   = lap_time(n);
                if (t < best - 1e-6f) {
                    best     = t;
                    o0       = cand;
                    improved = 1;
                } else {
                    rs_off[i] = o0; /* revert */
                }
            }
        }
        if (!improved) delta *= 0.5f; /* no gain at this scale, refine */
    }
}


/* Public entry point: build the racing line into track->points. */
void path_plan(Track *track)
{
    int i;
    int n_left  = track->left_count;
    int n_right = track->right_count;

    for (i = 0; i < n_left; i++) {
        left_x[i] = track->left_cones[i].x;
        left_y[i] = track->left_cones[i].y;
    }
    for (i = 0; i < n_right; i++) {
        right_x[i] = track->right_cones[i].x;
        right_y[i] = track->right_cones[i].y;
    }

    // 1. pair left/right cones into gates
    extract_gates(n_left, n_right);
    // 2. midpoint centreline and half-widths
    build_centreline();
    // 3. resample to uniform spacing
    resample_centreline();
    // 4. unit normals for lateral offset
    compute_normals();
    // 5. coordinate descent on lap time to find the racing line
    optimize_racing_line();

    for (i = 0; i < rs_n; i++) {
        track->points[i].x = rs_px(i);
        track->points[i].y = rs_py(i);
    }
    track->count = rs_n;
}
