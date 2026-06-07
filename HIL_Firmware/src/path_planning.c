#include "../include/path_planning.h"
#include <math.h>

/*
 * path_planning.c
 *
 * Stage 1: Mutual nearest-neighbour gate detection
 * -------------------------------------------------
 * For every left cone, find its nearest right cone (and vice versa).
 * A gate is accepted when the pairing is mutual AND the gate width is
 * within MAX_GATE_WIDTH_M.  This guarantees one gate per track section
 * and eliminates the long diagonal pairs produced by Delaunay at hairpins
 * and track crossings.
 *
 * Gates are then chain-sorted: starting from the gate nearest the track
 * start, each next gate is whichever unvisited gate midpoint is closest
 * to the current one.
 *
 * Stage 2: Minimum-curvature racing line (coordinate-descent + ternary search)
 * -----------------------------------------------------------------------------
 * Each gate i has a path point P_i = L_i + alpha_i * (R_i - L_i).
 * Coordinate descent: for each gate, fix all neighbours and use ternary
 * search to find the alpha_i that minimises the Menger curvature at P_i.
 * alpha is clamped to [RACING_MARGIN, 1 - RACING_MARGIN] so the car
 * always stays clear of the boundary cones.
 */


/* ------------------------------------------------------------------ */
/* Cone position arrays (loaded from track struct)                     */
/* ------------------------------------------------------------------ */

static float left_x[MAX_CONES],  left_y[MAX_CONES];
static float right_x[MAX_CONES], right_y[MAX_CONES];


/* ------------------------------------------------------------------ */
/* Gate extraction — mutual nearest-neighbour pairing + chain sort     */
/* ------------------------------------------------------------------ */

#define PP_MAX_GATES     400
#define MAX_GATE_WIDTH_M 10.0f   /* reject pairs wider than this (metres) */

typedef struct {
    TrackPoint left;
    TrackPoint right;
} Gate;

static Gate pp_gates[PP_MAX_GATES];
static Gate pp_gates_tmp[PP_MAX_GATES];
static int  pp_n_gates;

/*
 * Reorder pp_gates[0..n-1] into a nearest-neighbour midpoint chain.
 * Seed from (seed_x, seed_y) — the midpoint of the first left+right cones.
 */
static void sort_gates_by_chain(int n, float seed_x, float seed_y)
{
    int   used[PP_MAX_GATES];
    int   i, j, best_idx, cur;
    float best_dist, dx, dy, d;

    for (i = 0; i < n; i++) used[i] = 0;

    /* Find gate closest to the seed (track start) */
    best_idx = 0; best_dist = 1e18f;
    for (i = 0; i < n; i++) {
        float mx = (pp_gates[i].left.x + pp_gates[i].right.x) * 0.5f;
        float my = (pp_gates[i].left.y + pp_gates[i].right.y) * 0.5f;
        dx = mx - seed_x; dy = my - seed_y;
        d  = dx*dx + dy*dy;
        if (d < best_dist) { best_dist = d; best_idx = i; }
    }

    pp_gates_tmp[0] = pp_gates[best_idx];
    used[best_idx]  = 1;
    cur = 0;

    for (i = 1; i < n; i++) {
        float cx = (pp_gates_tmp[cur].left.x + pp_gates_tmp[cur].right.x) * 0.5f;
        float cy = (pp_gates_tmp[cur].left.y + pp_gates_tmp[cur].right.y) * 0.5f;
        best_idx  = -1;
        best_dist = 1e18f;
        for (j = 0; j < n; j++) {
            if (used[j]) continue;
            float mx = (pp_gates[j].left.x + pp_gates[j].right.x) * 0.5f;
            float my = (pp_gates[j].left.y + pp_gates[j].right.y) * 0.5f;
            dx = mx - cx; dy = my - cy;
            d  = dx*dx + dy*dy;
            if (d < best_dist) { best_dist = d; best_idx = j; }
        }
        if (best_idx < 0) break;
        pp_gates_tmp[i] = pp_gates[best_idx];
        used[best_idx]  = 1;
        cur = i;
    }

    for (i = 0; i < n; i++) pp_gates[i] = pp_gates_tmp[i];
}

static void extract_gates(int n_left, int n_right)
{
    int   i, j;
    int   nearest_right[MAX_CONES];
    int   nearest_left[MAX_CONES];
    float dx, dy, d, best_d;
    float max_w2 = MAX_GATE_WIDTH_M * MAX_GATE_WIDTH_M;

    /* For each left cone: index of nearest right cone */
    for (i = 0; i < n_left; i++) {
        nearest_right[i] = 0; best_d = 1e18f;
        for (j = 0; j < n_right; j++) {
            dx = right_x[j] - left_x[i];
            dy = right_y[j] - left_y[i];
            d  = dx*dx + dy*dy;
            if (d < best_d) { best_d = d; nearest_right[i] = j; }
        }
    }

    /* For each right cone: index of nearest left cone */
    for (j = 0; j < n_right; j++) {
        nearest_left[j] = 0; best_d = 1e18f;
        for (i = 0; i < n_left; i++) {
            dx = left_x[i] - right_x[j];
            dy = left_y[i] - right_y[j];
            d  = dx*dx + dy*dy;
            if (d < best_d) { best_d = d; nearest_left[j] = i; }
        }
    }

    pp_n_gates = 0;
    for (i = 0; i < n_left && pp_n_gates < PP_MAX_GATES; i++) {
        int ri = nearest_right[i];
        if (nearest_left[ri] != i) continue;  /* not mutual */

        dx = right_x[ri] - left_x[i];
        dy = right_y[ri] - left_y[i];
        if (dx*dx + dy*dy > max_w2) continue;  /* too wide */

        pp_gates[pp_n_gates].left.x  = left_x[i];
        pp_gates[pp_n_gates].left.y  = left_y[i];
        pp_gates[pp_n_gates].right.x = right_x[ri];
        pp_gates[pp_n_gates].right.y = right_y[ri];
        pp_n_gates++;
    }

    float seed_x = (left_x[0] + right_x[0]) * 0.5f;
    float seed_y = (left_y[0] + right_y[0]) * 0.5f;
    sort_gates_by_chain(pp_n_gates, seed_x, seed_y);
}


/* ------------------------------------------------------------------ */
/* Minimum-curvature racing line (coordinate-descent + ternary search) */
/* ------------------------------------------------------------------ */

static float menger(float ax, float ay,
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

static float opt_alpha(
    float px0, float py0,
    float lx,  float ly,
    float rx,  float ry,
    float px2, float py2,
    float lo,  float hi)
{
    float dx = rx - lx, dy = ry - ly;
    int   iter;

    for (iter = 0; iter < 30; iter++) {
        float m1 = lo + (hi - lo) / 3.0f;
        float m2 = hi - (hi - lo) / 3.0f;
        float k1 = menger(px0, py0, lx + m1*dx, ly + m1*dy, px2, py2);
        float k2 = menger(px0, py0, lx + m2*dx, ly + m2*dy, px2, py2);
        if (k1 < k2) hi = m2;
        else          lo = m1;
    }
    return (lo + hi) * 0.5f;
}

#define RACING_MARGIN  0.15f
#define OPT_PASSES     80

static float pp_alpha[PP_MAX_GATES];

static void optimize_racing_line(void)
{
    int pass, i;
    int n = pp_n_gates;

    for (i = 0; i < n; i++) pp_alpha[i] = 0.5f;

    for (pass = 0; pass < OPT_PASSES; pass++) {
        int start = (pass % 2 == 0) ? 0     : n - 1;
        int end   = (pass % 2 == 0) ? n     : -1;
        int step  = (pass % 2 == 0) ? 1     : -1;

        for (i = start; i != end; i += step) {
            int im1 = (i - 1 + n) % n;
            int ip1 = (i + 1)     % n;

            float px0 = pp_gates[im1].left.x + pp_alpha[im1] * (pp_gates[im1].right.x - pp_gates[im1].left.x);
            float py0 = pp_gates[im1].left.y + pp_alpha[im1] * (pp_gates[im1].right.y - pp_gates[im1].left.y);
            float px2 = pp_gates[ip1].left.x + pp_alpha[ip1] * (pp_gates[ip1].right.x - pp_gates[ip1].left.x);
            float py2 = pp_gates[ip1].left.y + pp_alpha[ip1] * (pp_gates[ip1].right.y - pp_gates[ip1].left.y);

            pp_alpha[i] = opt_alpha(
                px0, py0,
                pp_gates[i].left.x,  pp_gates[i].left.y,
                pp_gates[i].right.x, pp_gates[i].right.y,
                px2, py2,
                RACING_MARGIN, 1.0f - RACING_MARGIN);
        }
    }
}


/* ------------------------------------------------------------------ */
/* Public entry point                                                   */
/* ------------------------------------------------------------------ */

void path_plan(Track *track)
{
    int i;
    int n_left  = track->left_count;
    int n_right = track->right_count;
    int n;

    for (i = 0; i < n_left;  i++) { left_x[i]  = track->left_cones[i].x;  left_y[i]  = track->left_cones[i].y; }
    for (i = 0; i < n_right; i++) { right_x[i] = track->right_cones[i].x; right_y[i] = track->right_cones[i].y; }

    extract_gates(n_left, n_right);
    optimize_racing_line();

    n = pp_n_gates < MAX_WAYPOINTS ? pp_n_gates : MAX_WAYPOINTS;
    for (i = 0; i < n; i++) {
        track->points[i].x = pp_gates[i].left.x + pp_alpha[i] * (pp_gates[i].right.x - pp_gates[i].left.x);
        track->points[i].y = pp_gates[i].left.y + pp_alpha[i] * (pp_gates[i].right.y - pp_gates[i].left.y);
    }
    track->count = n;
}
