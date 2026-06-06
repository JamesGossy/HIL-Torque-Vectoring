#include "../include/path_planning.h"
#include <math.h>
#include <stdlib.h>

/*
 * path_planning.c
 *
 * Stage 1: Bowyer-Watson Delaunay triangulation
 * -----------------------------------------------
 * All left cones (indices 0..left_count-1) and right cones
 * (indices left_count..left_count+right_count-1) are triangulated together.
 * Edges that cross from a left cone to a right cone define the gates
 * the car must drive through.  Their endpoints (one left, one right cone)
 * are sorted by approximate track position and stored as gate_left / gate_right.
 *
 * Compared to simple index-matched pairing, the triangulation finds the
 * geometrically correct pairs: at hairpins the inner-side cones are more
 * densely spaced than the outer-side cones, so index i on the left is not
 * geometrically opposite index i on the right.  Delaunay cross-edges
 * always connect spatially adjacent cone pairs.
 *
 * Stage 2: Minimum-curvature optimisation
 * ----------------------------------------
 * Each gate i has a path point P_i = L_i + alpha_i * (R_i - L_i).
 * Coordinate descent: for each gate, fix all neighbours and use ternary
 * search to find the alpha_i that minimises the Menger curvature at P_i.
 * alpha is clamped to [RACING_MARGIN, 1 - RACING_MARGIN] so the car
 * always stays clear of the boundary cones.
 *
 * The result is a "geometric racing line" — wider entries, tighter apices,
 * wider exits — which raises the speed limit from the curvature-based
 * speed planner and requires less steering correction from Stanley.
 */

/* ------------------------------------------------------------------ */
/* Delaunay triangulation (Bowyer-Watson)                               */
/* ------------------------------------------------------------------ */

#define DT_MAX_PTS   (2 * MAX_CONES + 3)  /* all cones + super-tri verts */
#define DT_MAX_TRIS  1200                  /* ≥ 2 * DT_MAX_PTS with margin */
#define DT_MAX_HOLE  256                   /* max hole-boundary edges / step */

typedef struct { int a, b, c; int bad; } DTri;
typedef struct { int p, q; }             DEdge;

static float dt_x[DT_MAX_PTS];
static float dt_y[DT_MAX_PTS];
static DTri  dt_tris[DT_MAX_TRIS];
static int   dt_n;
static DEdge dt_hole[DT_MAX_HOLE];
static int   dt_hole_n;

/*
 * Returns non-zero if (px, py) is strictly inside the circumcircle of
 * triangle (a, b, c).  Handles both CW and CCW winding.
 */
static int in_circumcircle(int a, int b, int c, float px, float py)
{
    float ax = dt_x[a]-px,  ay = dt_y[a]-py;
    float bx = dt_x[b]-px,  by = dt_y[b]-py;
    float cx = dt_x[c]-px,  cy = dt_y[c]-py;

    float det = ax*(by*(cx*cx+cy*cy) - cy*(bx*bx+by*by))
              - ay*(bx*(cx*cx+cy*cy) - cx*(bx*bx+by*by))
              + (ax*ax+ay*ay)*(bx*cy - by*cx);

    /* winding: positive = CCW */
    float wnd = (dt_x[b]-dt_x[a])*(dt_y[c]-dt_y[a])
              - (dt_y[b]-dt_y[a])*(dt_x[c]-dt_x[a]);

    return (wnd > 0.0f) ? (det > 0.0f) : (det < 0.0f);
}

static void bw_init_super(int n)
{
    int i;
    float mnx = 1e9f, mxx = -1e9f, mny = 1e9f, mxy = -1e9f;
    float dm, mx, my;

    for (i = 0; i < n; i++) {
        if (dt_x[i] < mnx) mnx = dt_x[i];
        if (dt_x[i] > mxx) mxx = dt_x[i];
        if (dt_y[i] < mny) mny = dt_y[i];
        if (dt_y[i] > mxy) mxy = dt_y[i];
    }
    dm = (mxx - mnx) > (mxy - mny) ? (mxx - mnx) : (mxy - mny);
    mx = (mnx + mxx) * 0.5f;
    my = (mny + mxy) * 0.5f;

    /* CCW super-triangle that contains all points */
    dt_x[n+0] = mx - 3.0f*dm;  dt_y[n+0] = my - dm;
    dt_x[n+1] = mx + 3.0f*dm;  dt_y[n+1] = my - dm;
    dt_x[n+2] = mx;             dt_y[n+2] = my + 3.0f*dm;

    dt_tris[0].a = n;  dt_tris[0].b = n+1;  dt_tris[0].c = n+2;
    dt_tris[0].bad = 0;
    dt_n = 1;
}

static void bw_insert(int pi)
{
    int i, j, k;
    dt_hole_n = 0;

    for (i = 0; i < dt_n; i++)
        dt_tris[i].bad = in_circumcircle(dt_tris[i].a, dt_tris[i].b, dt_tris[i].c,
                                          dt_x[pi], dt_y[pi]);

    for (i = 0; i < dt_n; i++) {
        int ev[3][2], e;
        if (!dt_tris[i].bad) continue;
        ev[0][0]=dt_tris[i].a; ev[0][1]=dt_tris[i].b;
        ev[1][0]=dt_tris[i].b; ev[1][1]=dt_tris[i].c;
        ev[2][0]=dt_tris[i].c; ev[2][1]=dt_tris[i].a;
        for (e = 0; e < 3; e++) {
            int p = ev[e][0], q = ev[e][1], shared = 0;
            for (k = 0; k < dt_n; k++) {
                if (k == i || !dt_tris[k].bad) continue;
                if ((dt_tris[k].a==p||dt_tris[k].b==p||dt_tris[k].c==p) &&
                    (dt_tris[k].a==q||dt_tris[k].b==q||dt_tris[k].c==q)) {
                    shared = 1; break;
                }
            }
            if (!shared && dt_hole_n < DT_MAX_HOLE) {
                dt_hole[dt_hole_n].p = p;
                dt_hole[dt_hole_n].q = q;
                dt_hole_n++;
            }
        }
    }

    for (j = 0, i = 0; i < dt_n; i++)
        if (!dt_tris[i].bad) dt_tris[j++] = dt_tris[i];
    dt_n = j;

    for (i = 0; i < dt_hole_n && dt_n < DT_MAX_TRIS; i++) {
        dt_tris[dt_n].a   = dt_hole[i].p;
        dt_tris[dt_n].b   = dt_hole[i].q;
        dt_tris[dt_n].c   = pi;
        dt_tris[dt_n].bad = 0;
        dt_n++;
    }
}

static void bw_remove_super(int n)
{
    int i, j = 0;
    for (i = 0; i < dt_n; i++) {
        if (dt_tris[i].a >= n || dt_tris[i].b >= n || dt_tris[i].c >= n) continue;
        dt_tris[j++] = dt_tris[i];
    }
    dt_n = j;
}


/* ------------------------------------------------------------------ */
/* Gate extraction — cross-edges sorted along the track                */
/* ------------------------------------------------------------------ */

#define PP_MAX_GATES 400

typedef struct {
    TrackPoint left;
    TrackPoint right;
    float      score;   /* track-order key for sorting */
} Gate;

static Gate  pp_gates[PP_MAX_GATES];
static int   pp_n_gates;

static int gate_cmp(const void *a, const void *b)
{
    float sa = ((const Gate *)a)->score;
    float sb = ((const Gate *)b)->score;
    return (sa > sb) - (sa < sb);
}

/*
 * Run Delaunay on the cones already loaded into dt_x/dt_y, then
 * extract cross-edges (left-to-right) and sort them along the track.
 * Populates pp_gates[0..pp_n_gates-1].
 */
static void extract_gates(int n_left, int n_right)
{
    int i, t, e;
    int n_total = n_left + n_right;
    float scale = (n_right > 0) ? ((float)n_left / (float)n_right) : 1.0f;

    bw_init_super(n_total);
    for (i = 0; i < n_total; i++) bw_insert(i);
    bw_remove_super(n_total);

    pp_n_gates = 0;

    for (t = 0; t < dt_n; t++) {
        int abc[3] = { dt_tris[t].a, dt_tris[t].b, dt_tris[t].c };
        for (e = 0; e < 3; e++) {
            int p = abc[e], q = abc[(e+1)%3];
            if (p > q) { int tmp = p; p = q; q = tmp; }  /* normalise p < q */

            /* Cross-edge: p in left range [0, n_left), q in right range [n_left, n_total) */
            if (p >= n_left || q < n_left) continue;

            int li = p;
            int ri = q - n_left;

            /* Midpoint for deduplication */
            float mx = (dt_x[li] + dt_x[n_left + ri]) * 0.5f;
            float my = (dt_y[li] + dt_y[n_left + ri]) * 0.5f;

            int dup = 0, m;
            for (m = 0; m < pp_n_gates; m++) {
                float ddx = (pp_gates[m].left.x + pp_gates[m].right.x) * 0.5f - mx;
                float ddy = (pp_gates[m].left.y + pp_gates[m].right.y) * 0.5f - my;
                if (ddx*ddx + ddy*ddy < 0.01f) { dup = 1; break; }
            }
            if (!dup && pp_n_gates < PP_MAX_GATES) {
                pp_gates[pp_n_gates].left.x  = dt_x[li];
                pp_gates[pp_n_gates].left.y  = dt_y[li];
                pp_gates[pp_n_gates].right.x = dt_x[n_left + ri];
                pp_gates[pp_n_gates].right.y = dt_y[n_left + ri];
                pp_gates[pp_n_gates].score   = (float)li + (float)ri * scale;
                pp_n_gates++;
            }
        }
    }

    qsort(pp_gates, pp_n_gates, sizeof(Gate), gate_cmp);
}


/* ------------------------------------------------------------------ */
/* Minimum-curvature racing line (coordinate-descent + ternary search) */
/* ------------------------------------------------------------------ */

/*
 * Menger curvature of the triangle (ax,ay)-(bx,by)-(cx,cy).
 * κ = 2|AB × BC| / (|AB| |BC| |CA|).  Returns 0 if any side is degenerate.
 */
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

/*
 * Ternary-search the alpha value (in [lo, hi]) for gate i that minimises
 * the Menger curvature at P_i(alpha) given fixed P_prev and P_next.
 */
static float opt_alpha(
    float px0, float py0,   /* P_prev */
    float lx,  float ly,    /* left cone of gate i */
    float rx,  float ry,    /* right cone of gate i */
    float px2, float py2,   /* P_next */
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

/*
 * How far (as a fraction of gate width) to stay from each boundary cone.
 * 0.15 = 15 % clearance on each side.
 */
#define RACING_MARGIN  0.15f
#define OPT_PASSES     80

static float pp_alpha[PP_MAX_GATES];

static void optimize_racing_line(void)
{
    int pass, i;
    int n = pp_n_gates;

    /* Initialise to centreline */
    for (i = 0; i < n; i++) pp_alpha[i] = 0.5f;

    for (pass = 0; pass < OPT_PASSES; pass++) {
        /* Alternate forward and backward passes to propagate curvature
         * information in both directions each outer iteration. */
        int start = (pass % 2 == 0) ? 0        : n - 1;
        int end   = (pass % 2 == 0) ? n        : -1;
        int step  = (pass % 2 == 0) ? 1        : -1;

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
    int n_total = n_left + n_right;

    /* Load cone positions into the Delaunay point array */
    for (i = 0; i < n_left;  i++) { dt_x[i]          = track->left_cones[i].x;  dt_y[i]          = track->left_cones[i].y; }
    for (i = 0; i < n_right; i++) { dt_x[n_left + i]  = track->right_cones[i].x; dt_y[n_left + i]  = track->right_cones[i].y; }
    (void)n_total;

    /* Stage 1: find gates via Delaunay cross-edges */
    extract_gates(n_left, n_right);

    /* Stage 2: minimise curvature at each gate */
    optimize_racing_line();

    /* Write the resulting path into the track */
    int n = pp_n_gates < MAX_WAYPOINTS ? pp_n_gates : MAX_WAYPOINTS;
    for (i = 0; i < n; i++) {
        track->points[i].x = pp_gates[i].left.x + pp_alpha[i] * (pp_gates[i].right.x - pp_gates[i].left.x);
        track->points[i].y = pp_gates[i].left.y + pp_alpha[i] * (pp_gates[i].right.y - pp_gates[i].left.y);
    }
    track->count = n;
}
