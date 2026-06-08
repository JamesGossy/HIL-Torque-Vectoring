#include "../include/path_planning.h"
#include <math.h>

/*
 * Three stages:
 *   1. Gate detection: pair each left cone with its nearest right cone.
 *   2. Centreline resampling: take gate midpoints and resample to uniform spacing.
 *   3. Minimum-curvature line: bend the line within the track corridor.
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


/* Minimum-curvature racing line.
 * Each point moves only across-track: P_i = C_i + off_i * N_i, clamped to the
 * corridor. We minimise bending energy with a [1,-4,6,-4,1] Gauss-Seidel
 * sweep, giving off_i = -(N_i . S_i) / 6. */
static float rs_px(int i) { return rs_x[i] + rs_off[i] * rs_nx[i]; }
static float rs_py(int i) { return rs_y[i] + rs_off[i] * rs_ny[i]; }

static void optimize_racing_line(void)
{
    int pass, i, n = rs_n;

    for (i = 0; i < n; i++) rs_off[i] = 0.0f;
    if (n < 5) return;

    for (pass = 0; pass < OPT_PASSES; pass++) {
        int start = (pass % 2 == 0) ? 0     : n - 1;
        int end   = (pass % 2 == 0) ? n     : -1;
        int step  = (pass % 2 == 0) ? 1     : -1;

        for (i = start; i != end; i += step) {
            int im2 = (i - 2 + n) % n;
            int im1 = (i - 1 + n) % n;
            int ip1 = (i + 1)     % n;
            int ip2 = (i + 2)     % n;

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
