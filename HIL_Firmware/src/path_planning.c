#include "../include/path_planning.h"
#include "vehicle_config.h" /* MAX_STEER_RAD, ACK_NOMINAL, apex_speed() etc. */
#include "grip_model.h"     /* PEAK_LAT_FLAT / peak_lat() - the single grip ref */
#include "tunables.h"       /* runtime-overridable racing-line gains */
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
static float left_x[MAX_CONES], left_y[MAX_CONES];
static float right_x[MAX_CONES], right_y[MAX_CONES];


/* Gate extraction: pair each left cone with its nearest right cone. */
#define PP_MAX_GATES     400
#define MAX_GATE_WIDTH_M 10.0f /* reject pairs wider than this (metres) */

typedef struct {
    TrackPoint left;
    TrackPoint right;
} Gate;

static Gate pp_gates[PP_MAX_GATES];
static int pp_n_gates;

static void extract_gates(int n_left, int n_right)
{
    int i, j;
    int nearest_right[MAX_CONES];
    float dx, dy, d, best_d;
    float max_w2 = MAX_GATE_WIDTH_M * MAX_GATE_WIDTH_M;

    /* For each left cone, find the nearest right cone. */
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

    /* One gate per left cone, in cone order, dropping pairs that are too wide.
     * Cones are already ordered along the track, so gates inherit that order. */
    pp_n_gates = 0;
    for (i = 0; i < n_left && pp_n_gates < PP_MAX_GATES; i++) {
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


/* Centreline and uniform resampling.
 * Resampling to even spacing avoids the uneven, overlapping gates you get from
 * one point per cone. */

#define RESAMPLE_SPACING_M 2.5f /* target waypoint spacing, metres        */

/* g_RACING_MARGIN keeps the line this fraction of the half-width off each
 * boundary. Keep it modest so tight apexes are not over-constrained, but it must
 * leave enough clearance that the tracker - which now follows the line closely -
 * does not ride the apex cones: a faithful tracker needs the line itself to
 * clear cones by more than the off-track threshold plus its own cross-track
 * error. It is the racing line's one runtime tunable (g_RACING_MARGIN) in
 * shared/tunables.c; the line's grip and radius floor are now DERIVED from
 * vehicle_config.h (PEAK_LAT_FLAT / peak_lat() and PP_MIN_RADIUS_M). */
#define OPT_PASSES 400

/* Centreline (one entry per gate, in order) */
static float cl_x[PP_MAX_GATES];
static float cl_y[PP_MAX_GATES];
static float cl_h[PP_MAX_GATES]; /* corridor half-width at this point       */

/* Resampled, uniformly spaced racing-line buffers */
static float rs_x[MAX_WAYPOINTS]; /* centreline (reference)                  */
static float rs_y[MAX_WAYPOINTS];
static float rs_h[MAX_WAYPOINTS];  /* half-width                              */
static float rs_nx[MAX_WAYPOINTS]; /* unit track normal                       */
static float rs_ny[MAX_WAYPOINTS];
static float rs_off[MAX_WAYPOINTS]; /* lateral offset along the normal         */
static int rs_n;

/* Build the centreline (gate midpoints + half-widths) from the gates. */
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

/* Resample the closed centreline to uniform spacing, interpolating position
 * and half-width. */
static void resample_centreline(void)
{
    int g = pp_n_gates;
    int i;
    float total = 0.0f;

    for (i = 0; i < g; i++) {
        int j = (i + 1) % g;
        total += sqrtf(
            (cl_x[j] - cl_x[i]) * (cl_x[j] - cl_x[i]) + (cl_y[j] - cl_y[i]) * (cl_y[j] - cl_y[i]));
    }

    int m = (int)(total / RESAMPLE_SPACING_M + 0.5f);
    if (m < 8) m = 8;
    if (m > MAX_WAYPOINTS) m = MAX_WAYPOINTS;
    float step = total / (float)m;

    /* O(m*g): re-walks from segment 0 per output point. Fine at track sizes. */
    int seg       = 0;
    float seg_acc = 0.0f;
    float seg_len = 0.0f;
    (void)seg;
    (void)seg_acc;
    (void)seg_len; /* initialised inside the loop */

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

/* Unit track normals from the resampled (uniform) centreline. */
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
        /* left-hand normal */
        rs_nx[i] = -ty / len;
        rs_ny[i] = tx / len;
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
 * The solver optimises against the car's FULL physical grip (peak_lat(), the
 * tyre limit plus downforce, well above the GRIP_USE-scaled budget the on-car
 * planner drives with) so the LINE GEOMETRY reflects what the car can really do.
 * The car still plans its speed conservatively (GRIP_USE < 1); only the shape of
 * the path comes from this solve. The grip is no longer a tunable - it is the
 * physically-derived peak from grip_model.h, kept feasible by the PP_MIN_RADIUS_M
 * floor below. */

/* Longitudinal accel/brake limits for the speed-profile passes, m/s^2. */
#ifndef PP_ACCEL_LON
#define PP_ACCEL_LON 6.0f
#endif
#ifndef PP_BRAKE_LON
#define PP_BRAKE_LON 9.0f
#endif
/* Top speed cap for the profile, m/s (matches the car's TARGET_SPEED_MS). */
#define PP_V_MAX 30.0f

/* Feasibility floor: the car's steering geometry gives a kinematic minimum
 * turning radius (full lock), but the radius it can actually HOLD AT SPEED is
 * larger - once the front tyre is loaded, understeer/slip mean too tight an apex
 * needs more than full steering lock, so the car saturates at the stop, washes
 * wide, and scrubs to a near-stall crawl at the hairpin. The floor is therefore
 * DERIVED, not tuned: the kinematic minimum (WHEELBASE / tan(full wheel angle),
 * the wheel angle being MAX_STEER_RAD scaled by the Ackermann ratio) opened by a
 * fixed dynamic factor that accounts for the understeer/slip the kinematic model
 * ignores. This reproduces the old empirically-tuned ~6 m floor from geometry
 * alone. lap_time() additionally lets fast corners run tighter where downforce
 * holds them (the r_speed term below), so this floor only binds the slow
 * (no-downforce) hairpins.
 *
 * The quasi-steady-state speed model does NOT know any of this on its own: it
 * reads a tight radius as a low corner speed and happily spikes the curvature.
 * lap_time() charges a steep penalty for any segment tighter than this floor, so
 * the optimiser never draws an un-holdable apex. */
/* Feasibility floor radius, m: the kinematic minimum (WHEELBASE / tan(full wheel
 * angle), the wheel angle being the steering reference g_MAX_STEER_RAD scaled by
 * the Ackermann ratio) opened by g_PP_RADIUS_FACTOR for the understeer/slip the
 * kinematic model ignores. Both inputs are tunable, so this is computed at runtime
 * rather than as a #define. */
static float pp_min_radius_m(void)
{
    float r_kin = WHEELBASE_M / tanf(g_MAX_STEER_RAD * ACK_NOMINAL);
    return g_PP_RADIUS_FACTOR * r_kin;
}
/* Penalty weight, seconds of fake lap time per (1/m) of curvature over the
 * floor. Large enough to dominate any real time gain a too-tight apex could
 * offer, so an infeasible radius is always rejected. */
#define PP_CURV_PENALTY 50.0f

/* Coordinate-descent schedule. */
#define CD_PASSES    60    /* anneal steps                              */
#define CD_DELTA0    0.60f /* initial offset perturbation, m            */
#define CD_DELTA_MIN 0.02f /* stop annealing here, m                    */

static float rs_px(int i)
{
    return rs_x[i] + rs_off[i] * rs_nx[i];
}
static float rs_py(int i)
{
    return rs_y[i] + rs_off[i] * rs_ny[i];
}

/* Discrete curvature at point i of the current (offset) line, 1/m. */
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

/* Scratch buffers for the lap-time evaluation (module-static to avoid large
 * stack frames; the solve is single-threaded and runs once). */
static float lt_ds[MAX_WAYPOINTS]; /* segment length i -> i+1, m       */
static float lt_v[MAX_WAYPOINTS];  /* speed profile at point i, m/s    */
static float lt_k[MAX_WAYPOINTS];  /* line curvature at point i, 1/m   */

/* Longitudinal accel available at speed v on a segment of curvature k, under the
 * friction circle a_lat^2 + a_lon^2 <= gg(v)^2. The lateral grip already spent
 * cornering (a_lat = v^2*k) eats the circle, so less is left to accelerate or
 * brake - exactly the coupling the on-car planner uses (motion_control.c
 * brake_decel_avail). a_cap bounds it by the powertrain limit (PP_ACCEL_LON for
 * traction-out, PP_BRAKE_LON for braking); floored so the sweep always
 * progresses. Passing the GG budget as speed-dependent (gg(v)) lets the fast
 * corners brake/accelerate harder on their downforce, the whole point of #1. */
static float lon_accel_avail(float v, float k, float a_cap)
{
    float gg     = peak_lat(v); /* line shaped for the full physical grip */
    float a_lat  = v * v * k;
    float budget = gg * gg - a_lat * a_lat;
    float a_lon  = (budget > 0.0f) ? sqrtf(budget) : 0.0f;
    if (a_lon > a_cap) a_lon = a_cap;
    if (a_lon < 0.5f) a_lon = 0.5f;
    return a_lon;
}

/*
 * Lap time of the current offset line under a quasi-steady-state speed model.
 *
 * Three limits combine, exactly as a real lap-time simulator:
 *   - apex cap:  v_i <= sqrt(PP_GRIP_ACCEL / kappa_i)  (lateral grip)
 *   - forward:   v_{i+1} <= sqrt(v_i^2 + 2 a_lon ds_i) (can't accelerate harder)
 *   - backward:  v_i     <= sqrt(v_{i+1}^2 + 2 a_brk ds_i) (must be able to brake)
 * The forward and backward longitudinal budgets are taken under the FRICTION
 * CIRCLE (lon_accel_avail): braking into a corner is limited by the lateral grip
 * already in use, forcing an earlier, harder brake on the straight before a tight
 * apex and rewarding a line that straightens the entry - this is the real
 * combined-grip behaviour the on-car planner drives with, so the line is now
 * scored against the same car model it is driven by (#1).
 *
 * The forward+backward passes around the closed loop make the profile depend on
 * what FOLLOWS each corner - that is what rewards a late apex onto a straight.
 */
static float lap_time(int n)
{
    int i, lap;

    /* segment lengths + apex (lateral-grip) speed cap, and per-point curvature
     * cached for the friction-circle longitudinal passes below. */
    for (i = 0; i < n; i++) {
        int j    = (i + 1) % n;
        float dx = rs_px(j) - rs_px(i);
        float dy = rs_py(j) - rs_py(i);
        lt_ds[i] = sqrtf(dx * dx + dy * dy);

        /* Apex speed under SPEED-DEPENDENT grip: downforce adds lateral grip with
         * v^2, so the fast corners hold more than the flat PP_GRIP_ACCEL alone.
         * apex_speed() solves v^2*k = PP_GRIP_ACCEL + AERO_GRIP_COEF*v^2 in closed
         * form. Shaping the line for the real (downforce-aware) grip is the point
         * of the optimiser - a gripless line gives away the fast corners. */
        float k = line_curvature(i, n);
        lt_k[i] = k;
        lt_v[i] = apex_speed(PEAK_LAT_FLAT, k, PP_V_MAX);
    }

    /* Forward (traction-out) and backward (braking) passes under the friction
     * circle. Two laps of each around the closed loop so the limits propagate
     * fully past the wrap. The longitudinal budget is evaluated at the segment's
     * own speed and curvature, so a corner being entered fast brakes earlier. */
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

    /* T = Sum ds_i / v_avg over each segment, plus a feasibility penalty for any
     * point whose radius is below the DYNAMICALLY holdable radius for this car.
     *
     * #2: the holdable radius shrinks with speed. The car's grip-limited minimum
     * radius is r = v^2 / a_lat_max(v): at the apex speed the tyre can hold, a
     * faster corner (more downforce) can hold a TIGHTER geometric radius. A flat
     * PP_MIN_RADIUS_M floors every corner at the SLOW-corner (no-downforce) limit,
     * needlessly opening the fast corners. We floor instead at the radius the car
     * can actually hold at this point's planned speed, so fast corners may run
     * tighter (carrying more apex speed) while the hairpin still gets its full
     * PP_MIN_RADIUS_M opening (no downforce there). */
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

/* Seed the offsets with the minimum-curvature line: a [1,-4,6,-4,1]
 * bending-energy Gauss-Seidel relaxation, off_i = -(N_i . S_i)/6, clamped to the
 * corridor. Smooth and feasible, so the lap-time search starts from a sane line
 * and only has to refine it (mainly shifting apexes earlier/later). */
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

static void optimize_racing_line(void)
{
    int pass, i, n = rs_n;

    for (i = 0; i < n; i++)
        rs_off[i] = 0.0f;
    if (n < 5) return;

    /* Warm start from the smooth minimum-curvature line. */
    seed_min_curvature(n);

    /* Coordinate descent on the lap-time objective. Each point tries to move
     * its cross-track offset by +/-delta; the move is kept only if it lowers the
     * whole-lap time. delta anneals geometrically so the search localises the
     * apexes coarsely first, then fine-tunes. lap_time() reads the live offsets,
     * so each accepted move is felt by its neighbours on the next visit. */
    float delta = CD_DELTA0;
    float best  = lap_time(n);

    for (pass = 0; pass < CD_PASSES && delta >= CD_DELTA_MIN; pass++) {
        int improved = 0;
        for (i = 0; i < n; i++) {
            float lim = (1.0f - 2.0f * g_RACING_MARGIN) * rs_h[i];
            float o0  = rs_off[i];

            /* try + then - */
            for (int s = 0; s < 2; s++) {
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
        if (!improved) delta *= 0.5f; /* no gain at this scale -> refine */
    }
}


/* Public entry point. */
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
