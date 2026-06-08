#include "../include/path_planning.h"
#include "parameters_config.h"   /* MAX_LATERAL_ACCEL_MS2 (self-consistent grip) */
#include <math.h>

/*
 * Three stages:
 *   1. Gate detection: pair each left cone with its nearest right cone.
 *   2. Centreline resampling: take gate midpoints and resample to uniform spacing.
 *   3. Minimum-TIME line: bend the line within the track corridor, weighting the
 *      bend toward the corners the car is actually slow in (see
 *      optimize_racing_line). Pure minimum-curvature hugs every apex equally,
 *      which forces an infeasibly tight radius at the hairpins; the speed
 *      planner then caps the car very slow there AND the tracker runs wide onto
 *      the apex cones. The min-time weighting opens those corners up instead.
 */

/* Cone positions, loaded from the track struct. */
static float left_x[MAX_CONES],  left_y[MAX_CONES];
static float right_x[MAX_CONES], right_y[MAX_CONES];


/* Gate extraction: pair each left cone with its nearest right cone. */
#define PP_MAX_GATES     400
#define MAX_GATE_WIDTH_M 10.0f   /* reject pairs wider than this (metres) */

typedef struct {
    TrackPoint left;
    TrackPoint right;
} Gate;

static Gate pp_gates[PP_MAX_GATES];
static int  pp_n_gates;

static void extract_gates(int n_left, int n_right)
{
    int   i, j;
    int   nearest_right[MAX_CONES];
    float dx, dy, d, best_d;
    float max_w2 = MAX_GATE_WIDTH_M * MAX_GATE_WIDTH_M;

    /* For each left cone, find the nearest right cone. */
    for (i = 0; i < n_left; i++) {
        nearest_right[i] = 0; best_d = 1e18f;
        for (j = 0; j < n_right; j++) {
            dx = right_x[j] - left_x[i];
            dy = right_y[j] - left_y[i];
            d  = dx*dx + dy*dy;
            if (d < best_d) { best_d = d; nearest_right[i] = j; }
        }
    }

    /* One gate per left cone, in cone order, dropping pairs that are too wide.
     * Cones are already ordered along the track, so gates inherit that order. */
    pp_n_gates = 0;
    for (i = 0; i < n_left && pp_n_gates < PP_MAX_GATES; i++) {
        int ri = nearest_right[i];
        dx = right_x[ri] - left_x[i];
        dy = right_y[ri] - left_y[i];
        if (dx*dx + dy*dy > max_w2) continue;

        pp_gates[pp_n_gates].left.x  = left_x[i];
        pp_gates[pp_n_gates].left.y  = left_y[i];
        pp_gates[pp_n_gates].right.x = right_x[ri];
        pp_gates[pp_n_gates].right.y = right_y[ri];
        pp_n_gates++;
    }
}


/* Centreline and uniform resampling.
 * Resampling to even spacing avoids the uneven, overlapping gates you get from
 * one point per cone. */

#define RESAMPLE_SPACING_M  2.5f   /* target waypoint spacing, metres        */

/* RACING_MARGIN keeps the line this fraction of the half-width off each
 * boundary. Keep it modest so tight apexes are not over-constrained. */
#define RACING_MARGIN  0.15f
#define OPT_PASSES     400

/* Centreline (one entry per gate, in order) */
static float cl_x[PP_MAX_GATES];
static float cl_y[PP_MAX_GATES];
static float cl_h[PP_MAX_GATES];   /* corridor half-width at this point       */

/* Resampled, uniformly spaced racing-line buffers */
static float rs_x[MAX_WAYPOINTS];  /* centreline (reference)                  */
static float rs_y[MAX_WAYPOINTS];
static float rs_h[MAX_WAYPOINTS];  /* half-width                              */
static float rs_nx[MAX_WAYPOINTS]; /* unit track normal                       */
static float rs_ny[MAX_WAYPOINTS];
static float rs_off[MAX_WAYPOINTS];/* lateral offset along the normal         */
static int   rs_n;

/* Build the centreline (gate midpoints + half-widths) from the gates. */
static void build_centreline(void)
{
    int i;
    for (i = 0; i < pp_n_gates; i++) {
        cl_x[i] = 0.5f * (pp_gates[i].left.x + pp_gates[i].right.x);
        cl_y[i] = 0.5f * (pp_gates[i].left.y + pp_gates[i].right.y);
        float dx = pp_gates[i].right.x - pp_gates[i].left.x;
        float dy = pp_gates[i].right.y - pp_gates[i].left.y;
        cl_h[i] = 0.5f * sqrtf(dx*dx + dy*dy);
    }
}

/* Resample the closed centreline to uniform spacing, interpolating position
 * and half-width. */
static void resample_centreline(void)
{
    int   g = pp_n_gates;
    int   i;
    float total = 0.0f;

    for (i = 0; i < g; i++) {
        int j = (i + 1) % g;
        total += sqrtf((cl_x[j]-cl_x[i])*(cl_x[j]-cl_x[i]) +
                       (cl_y[j]-cl_y[i])*(cl_y[j]-cl_y[i]));
    }

    int m = (int)(total / RESAMPLE_SPACING_M + 0.5f);
    if (m < 8)               m = 8;
    if (m > MAX_WAYPOINTS)   m = MAX_WAYPOINTS;
    float step = total / (float)m;

    /* O(m*g): re-walks from segment 0 per output point. Fine at track sizes. */
    int   seg     = 0;
    float seg_acc = 0.0f;
    float seg_len = 0.0f;
    (void)seg; (void)seg_acc; (void)seg_len; /* initialised inside the loop */

    for (i = 0; i < m; i++) {
        float target = i * step;
        float walked = 0.0f;
        seg = 0; seg_acc = 0.0f;
        while (seg < g) {
            int j = (seg + 1) % g;
            seg_len = sqrtf((cl_x[j]-cl_x[seg])*(cl_x[j]-cl_x[seg]) +
                            (cl_y[j]-cl_y[seg])*(cl_y[j]-cl_y[seg]));
            if (walked + seg_len >= target || seg == g - 1) {
                seg_acc = target - walked;
                break;
            }
            walked += seg_len;
            seg++;
        }
        int   j = (seg + 1) % g;
        float t = (seg_len > 1e-6f) ? (seg_acc / seg_len) : 0.0f;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        rs_x[i] = cl_x[seg] + t * (cl_x[j] - cl_x[seg]);
        rs_y[i] = cl_y[seg] + t * (cl_y[j] - cl_y[seg]);
        rs_h[i] = cl_h[seg] + t * (cl_h[j] - cl_h[seg]);
    }
    rs_n = m;
}

/* Unit track normals from the resampled (uniform) centreline. */
static void compute_normals(void)
{
    int i, n = rs_n;
    for (i = 0; i < n; i++) {
        int p = (i - 1 + n) % n;
        int q = (i + 1) % n;
        float tx = rs_x[q] - rs_x[p];
        float ty = rs_y[q] - rs_y[p];
        float len = sqrtf(tx*tx + ty*ty);
        if (len < 1e-6f) { rs_nx[i] = 0.0f; rs_ny[i] = 0.0f; continue; }
        /* left-hand normal */
        rs_nx[i] = -ty / len;
        rs_ny[i] =  tx / len;
    }
}


/* ====================================================================== *
 *  Minimum-LAP-TIME racing line                                          *
 * ====================================================================== *
 *
 * The previous lines (centreline, minimum-curvature, minimum-time-weighted
 * curvature) are all PURELY GEOMETRIC: they shape the path from curvature alone
 * and never see the car's speed. The fastest line is not the lowest-curvature
 * one - it is the one that minimises the time T = Sum ds_i / v_i, where v_i is
 * the speed the car can actually carry at that point.
 *
 * That coupling is what produces real racing behaviour the geometric lines
 * cannot:
 *   - out-in-out: enter wide, touch the apex, exit wide (max radius = max speed)
 *   - LATE APEX: when a straight follows a corner, sacrifice apex speed to
 *     straighten the EXIT, because carrying speed onto a long straight is worth
 *     more time than a faster apex. A geometric line is symmetric and blind to
 *     what follows the corner; a lap-time objective is not.
 *
 * Method (quasi-steady-state lap-time model + coordinate descent):
 *   1. Seed the offsets with the minimum-curvature line (a good, smooth start so
 *      the local search converges fast and avoids silly minima).
 *   2. lap_time(): from the candidate offsets, build the speed profile with a
 *      forward+backward pass under the friction circle (apex cap from lateral
 *      grip, forward accel limit, backward brake limit), then sum ds/v.
 *   3. Coordinate descent: nudge each point's offset +/-delta along its normal,
 *      keep the nudge if it lowers lap time, anneal delta over passes.
 *
 * Runs once at startup, so the O(passes * n^2) cost is irrelevant.
 *
 * The solver optimises against PP_GRIP_ACCEL (the car's TRUE grip, well above
 * the conservative MAX_LATERAL_ACCEL_MS2 the on-car planner drives with) so the
 * LINE GEOMETRY reflects what the car can really do. The car still plans its
 * speed conservatively; only the shape of the path comes from this solve. */

/* Combined grip the solver shapes the line for, m/s^2. By default this is the
 * SAME budget the on-car speed planner uses (MAX_LATERAL_ACCEL_MS2), so the line
 * is time-optimal for the car that actually drives it - not for a faster
 * hypothetical car. (An earlier version set this well above the planner budget;
 * that only "worked" because a high grip masked the solver spiking the curvature
 * into an un-steerable apex. The PP_MIN_RADIUS_M feasibility floor below now
 * prevents that directly, so the grip can be self-consistent.) Override it only
 * if you deliberately want to shape a more/less aggressive line than the car
 * drives; always confirm 0 off-track with `make eval`. */
#ifndef PP_GRIP_ACCEL
#define PP_GRIP_ACCEL     MAX_LATERAL_ACCEL_MS2
#endif
/* Longitudinal accel/brake limits for the speed-profile passes, m/s^2. */
#ifndef PP_ACCEL_LON
#define PP_ACCEL_LON      6.0f
#endif
#ifndef PP_BRAKE_LON
#define PP_BRAKE_LON      9.0f
#endif
/* Top speed cap for the profile, m/s (matches the car's TARGET_SPEED_MS). */
#define PP_V_MAX          30.0f

/* Feasibility floor: the car's steering geometry gives a minimum turning radius
 * of ~3.8 m (MAX_STEER_RAD), so any line tighter than that is physically
 * un-followable - the tracker cannot steer hard enough and runs wide off the
 * line. The quasi-steady-state speed model on its own does NOT know this: it
 * just reads a very tight radius as a very low corner speed and is happy to
 * shave a few hundredths there, spiking the local curvature into a corner the
 * car cannot make. PP_MIN_RADIUS_M is a hard floor a little above the geometric
 * R_min (margin for the tracker); lap_time() charges a steep time penalty for
 * any segment tighter than it, so the optimiser never chooses an un-steerable
 * apex. This - not a high PP_GRIP_ACCEL - is what keeps the line followable. */
#ifndef PP_MIN_RADIUS_M
#define PP_MIN_RADIUS_M   4.5f
#endif
/* Penalty weight, seconds of fake lap time per (1/m) of curvature over the
 * floor. Large enough to dominate any real time gain a too-tight apex could
 * offer, so an infeasible radius is always rejected. */
#define PP_CURV_PENALTY   50.0f

/* Coordinate-descent schedule. */
#define CD_PASSES         60      /* anneal steps                              */
#define CD_DELTA0         0.60f   /* initial offset perturbation, m            */
#define CD_DELTA_MIN      0.02f   /* stop annealing here, m                    */

static float rs_px(int i) { return rs_x[i] + rs_off[i] * rs_nx[i]; }
static float rs_py(int i) { return rs_y[i] + rs_off[i] * rs_ny[i]; }

/* Discrete curvature at point i of the current (offset) line, 1/m. */
static float line_curvature(int i, int n)
{
    int im1 = (i - 1 + n) % n;
    int ip1 = (i + 1)     % n;
    float ax = rs_px(im1), ay = rs_py(im1);
    float bx = rs_px(i),   by = rs_py(i);
    float cx = rs_px(ip1), cy = rs_py(ip1);
    float abx = bx-ax, aby = by-ay;
    float bcx = cx-bx, bcy = cy-by;
    float cax = ax-cx, cay = ay-cy;
    float ab = sqrtf(abx*abx + aby*aby);
    float bc = sqrtf(bcx*bcx + bcy*bcy);
    float ca = sqrtf(cax*cax + cay*cay);
    float crs = fabsf(abx*bcy - aby*bcx);
    if (ab < 0.01f || bc < 0.01f || ca < 0.01f || crs < 1e-6f) return 0.0f;
    return 2.0f * crs / (ab * bc * ca);
}

/* Scratch buffers for the lap-time evaluation (module-static to avoid large
 * stack frames; the solve is single-threaded and runs once). */
static float lt_ds[MAX_WAYPOINTS];  /* segment length i -> i+1, m   */
static float lt_v [MAX_WAYPOINTS];  /* speed profile at point i, m/s */

/*
 * Lap time of the current offset line under a quasi-steady-state speed model.
 *
 * Three limits combine, exactly as a real lap-time simulator:
 *   - apex cap:  v_i <= sqrt(PP_GRIP_ACCEL / kappa_i)  (lateral grip)
 *   - forward:   v_{i+1} <= sqrt(v_i^2 + 2 a_lon ds_i) (can't accelerate harder)
 *   - backward:  v_i     <= sqrt(v_{i+1}^2 + 2 a_brk ds_i) (must be able to brake)
 * The forward+backward passes around the closed loop make the profile depend on
 * what FOLLOWS each corner - that is what rewards a late apex onto a straight.
 */
static float lap_time(int n)
{
    int i, lap;

    /* segment lengths + apex (lateral-grip) speed cap */
    for (i = 0; i < n; i++) {
        int j = (i + 1) % n;
        float dx = rs_px(j) - rs_px(i);
        float dy = rs_py(j) - rs_py(i);
        lt_ds[i] = sqrtf(dx*dx + dy*dy);

        float k = line_curvature(i, n);
        float vcap = (k > 1e-4f) ? sqrtf(PP_GRIP_ACCEL / k) : PP_V_MAX;
        lt_v[i] = (vcap < PP_V_MAX) ? vcap : PP_V_MAX;
    }

    /* Forward (traction-out) and backward (braking) passes. Two laps of each
     * around the closed loop so the limits propagate fully past the wrap. */
    for (lap = 0; lap < 2; lap++) {
        for (i = 0; i < n; i++) {
            int j = (i + 1) % n;
            float reach = sqrtf(lt_v[i]*lt_v[i] + 2.0f*PP_ACCEL_LON*lt_ds[i]);
            if (lt_v[j] > reach) lt_v[j] = reach;
        }
        for (i = n - 1; i >= 0; i--) {
            int j = (i + 1) % n;
            float reach = sqrtf(lt_v[j]*lt_v[j] + 2.0f*PP_BRAKE_LON*lt_ds[i]);
            if (lt_v[i] > reach) lt_v[i] = reach;
        }
    }

    /* T = Sum ds_i / v_avg over each segment, plus a feasibility penalty for any
     * point whose radius is below PP_MIN_RADIUS_M (un-steerable for this car). */
    const float k_floor = 1.0f / PP_MIN_RADIUS_M;
    float t = 0.0f;
    for (i = 0; i < n; i++) {
        int j = (i + 1) % n;
        float vavg = 0.5f * (lt_v[i] + lt_v[j]);
        if (vavg < 0.5f) vavg = 0.5f;     /* guard near standstill */
        t += lt_ds[i] / vavg;

        float k = line_curvature(i, n);
        if (k > k_floor)
            t += PP_CURV_PENALTY * (k - k_floor);
    }
    return t;
}

/* Seed the offsets with the minimum-curvature line: a [1,-4,6,-4,1]
 * bending-energy Gauss-Seidel relaxation, off_i = -(N_i . S_i)/6, clamped to the
 * corridor. Smooth and feasible, so the lap-time search starts from a sane line
 * and only has to refine it (mainly shifting apexes earlier/later). */
static void seed_min_curvature(int n)
{
    int pass, i;
    for (i = 0; i < n; i++) rs_off[i] = 0.0f;

    for (pass = 0; pass < OPT_PASSES; pass++) {
        int start = (pass % 2 == 0) ? 0 : n - 1;
        int end   = (pass % 2 == 0) ? n : -1;
        int step  = (pass % 2 == 0) ? 1 : -1;
        for (i = start; i != end; i += step) {
            int im2 = (i - 2 + n) % n, im1 = (i - 1 + n) % n;
            int ip1 = (i + 1) % n,     ip2 = (i + 2) % n;
            float Sx = rs_px(im2) - 4.0f*rs_px(im1) + 6.0f*rs_x[i] - 4.0f*rs_px(ip1) + rs_px(ip2);
            float Sy = rs_py(im2) - 4.0f*rs_py(im1) + 6.0f*rs_y[i] - 4.0f*rs_py(ip1) + rs_py(ip2);
            float o = -(rs_nx[i]*Sx + rs_ny[i]*Sy) / 6.0f;
            float lim = (1.0f - 2.0f*RACING_MARGIN) * rs_h[i];
            if (o >  lim) o =  lim;
            if (o < -lim) o = -lim;
            rs_off[i] = o;
        }
    }
}

static void optimize_racing_line(void)
{
    int pass, i, n = rs_n;

    for (i = 0; i < n; i++) rs_off[i] = 0.0f;
    if (n < 5) return;

    /* Warm start from the smooth minimum-curvature line. */
    seed_min_curvature(n);

    /* Coordinate descent on the lap-time objective. Each point tries to move
     * its cross-track offset by +/-delta; the move is kept only if it lowers the
     * whole-lap time. delta anneals geometrically so the search localises the
     * apexes coarsely first, then fine-tunes. lap_time() reads the live offsets,
     * so each accepted move is felt by its neighbours on the next visit. */
    float delta = CD_DELTA0;
    float best = lap_time(n);

    for (pass = 0; pass < CD_PASSES && delta >= CD_DELTA_MIN; pass++) {
        int improved = 0;
        for (i = 0; i < n; i++) {
            float lim = (1.0f - 2.0f*RACING_MARGIN) * rs_h[i];
            float o0  = rs_off[i];

            /* try + then - */
            for (int s = 0; s < 2; s++) {
                float cand = o0 + (s == 0 ? delta : -delta);
                if (cand >  lim) cand =  lim;
                if (cand < -lim) cand = -lim;
                if (cand == rs_off[i]) continue;

                rs_off[i] = cand;
                float t = lap_time(n);
                if (t < best - 1e-6f) {
                    best = t; o0 = cand; improved = 1;
                } else {
                    rs_off[i] = o0;   /* revert */
                }
            }
        }
        if (!improved) delta *= 0.5f;   /* no gain at this scale -> refine */
    }
}


/* Public entry point. */
void path_plan(Track *track)
{
    int i;
    int n_left  = track->left_count;
    int n_right = track->right_count;

    for (i = 0; i < n_left;  i++) { left_x[i]  = track->left_cones[i].x;  left_y[i]  = track->left_cones[i].y; }
    for (i = 0; i < n_right; i++) { right_x[i] = track->right_cones[i].x; right_y[i] = track->right_cones[i].y; }

    extract_gates(n_left, n_right);
    build_centreline();
    resample_centreline();
    compute_normals();
    optimize_racing_line();

    for (i = 0; i < rs_n; i++) {
        track->points[i].x = rs_px(i);
        track->points[i].y = rs_py(i);
    }
    track->count = rs_n;
}
