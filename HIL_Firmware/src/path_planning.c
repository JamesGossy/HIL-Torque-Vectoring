#include "../include/path_planning.h"
#include <math.h>

/*
 * path_planning.c
 *
 * Stage 1: Gate detection
 * -----------------------
 * One gate per left cone, in cone order (the cone arrays are already ordered
 * along the track), paired to its nearest right cone and filtered by gate
 * width.  See extract_gates() for why this beats mutual-nearest-neighbour
 * pairing at hairpins.
 *
 * Stage 2: Centreline + uniform resampling
 * ----------------------------------------
 * The gate midpoints form a centreline (with a local corridor half-width).
 * That centreline is resampled to uniform arc-length spacing, which removes the
 * uneven spacing and overlapping-gate artefacts of one-point-per-cone (those
 * caused the start-straight spike).  See resample_centreline().
 *
 * Stage 3: Minimum-curvature racing line (global bending-energy minimisation)
 * --------------------------------------------------------------------------
 * Each uniform point carries a lateral offset along the track normal,
 * P_i = C_i + off_i * N_i.  We minimise sum_i ||P_{i-1} - 2 P_i + P_{i+1}||^2 by
 * sweeping the [1,-4,6,-4,1] Gauss-Seidel update over off, clamped to the
 * corridor.  See optimize_racing_line() for the derivation.
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
static int  pp_n_gates;

static void extract_gates(int n_left, int n_right)
{
    int   i, j;
    int   nearest_right[MAX_CONES];
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

    /*
     * Build one gate per LEFT cone, in the order the cones are stored (the cone
     * arrays are already ordered along the track).  Each left cone is paired to
     * its nearest right cone, filtered by gate width.
     *
     * The previous mutual-nearest-neighbour rule was wrong at hairpins: there
     * the tight (inside) boundary has few, bunched cones while the open side
     * has many, so several outside cones share one inside apex cone.  Only one
     * pairing can be mutual, so the apex cones were dropped and the racing line
     * chorded straight across the corner.
     *
     * Anchoring on every left cone in cone order fixes both failure modes
     * without any reordering:
     *   - where the left side is the inside of a corner, each apex cone still
     *     gets its own gate, so the line follows the apex;
     *   - where the left side is the outside, the many outside cones all pair to
     *     the inside apex cone and naturally fan around it.
     * Because the gates inherit the cone ordering, no chain sort is needed and
     * the racing line cannot become tangled (which a greedy nearest-neighbour
     * sort can do once gates are dense).  The width filter still rejects long
     * diagonal pairs.
     */
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


/* ------------------------------------------------------------------ */
/* Centreline, uniform resampling                                       */
/* ------------------------------------------------------------------ */

/*
 * Why resample instead of using one point per gate:
 *
 * One point per cone gives wildly uneven spacing (0.3 m to 5 m) and, where the
 * two boundaries have different cone counts, two adjacent cones can map to the
 * SAME opposite cone — producing overlapping gates that make the line zig-zag
 * (the start-straight spike).  Resampling the centreline to a uniform spacing
 * removes both problems: the racing line is then defined by evenly spaced
 * points whose corridor half-width is interpolated, independent of where the
 * individual cones happen to sit.
 */

#define RESAMPLE_SPACING_M  2.5f   /* target waypoint spacing, metres        */

/*
 * RACING_MARGIN keeps the racing line at least this fraction of the local
 * track half-width away from each boundary.  Keep it modest: a large margin
 * over-constrains tight apexes.
 */
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

/*
 * Resample the closed centreline to uniform arc-length spacing, interpolating
 * position and half-width.  Fills rs_x/rs_y/rs_h and rs_n.
 */
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

    int   seg     = 0;        /* current centreline segment index            */
    float seg_acc = 0.0f;     /* arc length consumed within the current seg   */
    float seg_len;
    {
        int j = 1 % g;
        seg_len = sqrtf((cl_x[j]-cl_x[0])*(cl_x[j]-cl_x[0]) +
                        (cl_y[j]-cl_y[0])*(cl_y[j]-cl_y[0]));
    }

    for (i = 0; i < m; i++) {
        float target = i * step;
        /* Walk segments until target arc length falls inside the current one */
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


/* ------------------------------------------------------------------ */
/* Minimum-curvature racing line (global bending-energy minimisation)   */
/* ------------------------------------------------------------------ */

/*
 * The line is P_i = C_i + off_i * N_i, where C_i is the uniform centreline
 * point and N_i its unit normal — so the single degree of freedom per point is
 * always across-track (no gate skew) and the line stays in the corridor as
 * long as |off_i| <= (1 - 2*RACING_MARGIN) * half_width.
 *
 * We minimise the bending energy E = sum_i ||P_{i-1} - 2 P_i + P_{i+1}||^2.
 * Setting dE/doff_i = 0 (neighbours fixed) gives the [1,-4,6,-4,1] stencil:
 * with S_i = P_{i-2} - 4 P_{i-1} + 6 C_i - 4 P_{i+1} + P_{i+2} and |N_i| = 1,
 *     off_i = -(N_i . S_i) / 6,
 * swept Gauss-Seidel to convergence.  Because the points are uniformly spaced
 * the stencil is well conditioned and the result is smooth and spike-free.
 */
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

            /* S_i = P_{i-2} - 4 P_{i-1} + 6 C_i - 4 P_{i+1} + P_{i+2} */
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


/* ------------------------------------------------------------------ */
/* Public entry point                                                   */
/* ------------------------------------------------------------------ */

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
