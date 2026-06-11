// Two-phase online planner. Phase 1 builds a short reactive centreline from the
// SLAM cone map ahead of the car and follows it slowly while mapping. Phase 2,
// after loop closure, runs the offline racing-line optimiser once on the full
// map and races it. Everything is driven by the SLAM map/pose, not ground truth.

#include "../include/online_planner.h"
#include "../include/path_planning.h"
#include "../include/motion_control.h"
#include "../../shared/tunables.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define LOCAL_FWD_M    16.0f /* how far ahead to gather cones for the local line */
#define LOCAL_MAX_GATE 10.0f /* reject L/R pairs wider than this, m */
#define BIG_SPEED      1e6f

static const float OP_PI = 3.14159265358979323846f;

void online_planner_init(OnlinePlanner *p)
{
    p->phase2_active        = 0;
    p->racing_line.count    = 0;
}

static float wrap_pi(float a)
{
    while (a > OP_PI)
        a -= 2.0f * OP_PI;
    while (a < -OP_PI)
        a += 2.0f * OP_PI;
    return a;
}

// A mapped cone projected into the car frame: forward distance and side offset.
typedef struct {
    float fwd;
    float x, y;
} LocalCone;

// Collect confident cones of one colour that lie ahead of the car, sorted by forward distance.
static int gather_ahead(const EcuMap *m, int left, float px, float py, float ch, LocalCone *out,
    int cap)
{
    const MapPoint *arr = left ? m->left_cones : m->right_cones;
    int n               = left ? m->left_count : m->right_count;
    float c = cosf(ch), s = sinf(ch);
    int k   = 0;
    for (int i = 0; i < n && k < cap; i++) {
        float dx  = arr[i].x - px, dy = arr[i].y - py;
        float fwd = dx * c + dy * s;      // along heading
        float lat = -dx * s + dy * c;     // left positive
        if (fwd < -2.0f || fwd > LOCAL_FWD_M) continue;
        if (fabsf(lat) > LOCAL_MAX_GATE) continue;
        out[k].fwd = fwd;
        out[k].x   = arr[i].x;
        out[k].y   = arr[i].y;
        k++;
    }
    // insertion sort by forward distance (small arrays)
    for (int i = 1; i < k; i++) {
        LocalCone t = out[i];
        int j       = i - 1;
        while (j >= 0 && out[j].fwd > t.fwd) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = t;
    }
    return k;
}

// A gate midpoint with its forward distance from the car.
typedef struct {
    float fwd, x, y;
} Mid;

// Order cones into a forward chain: start nearest the car, then repeatedly take
// the nearest unused cone to the last one. Gives along-track order so pairing by
// index produces correct corridor midpoints (nearest-cone pairing skews in corners).
static void chain_order(LocalCone *c, int n)
{
    if (n < 2) return;
    for (int i = 1; i < n; i++) {
        int best = i;
        float bd = 1e18f;
        for (int j = i; j < n; j++) {
            float dx = c[j].x - c[i - 1].x, dy = c[j].y - c[i - 1].y;
            float d2 = dx * dx + dy * dy;
            if (d2 < bd) { bd = d2; best = j; }
        }
        LocalCone t = c[i];
        c[i]        = c[best];
        c[best]     = t;
    }
}

// Build a short centreline ahead of (px,py,ch) from gate midpoints. Robust to
// missing/jittery cones: it forms a gate for every left cone (nearest right) and
// every right cone (nearest left), dedups midpoints, sorts by forward distance,
// and keeps a strictly forward-advancing chain anchored at the car. Returns the
// number of points written to m->points.
static int build_local_centreline(EcuMap *m, float px, float py, float ch)
{
    static LocalCone L[ECU_MAX_CONES], R[ECU_MAX_CONES];
    int nl = gather_ahead(m, 1, px, py, ch, L, ECU_MAX_CONES);
    int nr = gather_ahead(m, 0, px, py, ch, R, ECU_MAX_CONES);

    // Order each side along-track so index pairing follows the corridor.
    chain_order(L, nl);
    chain_order(R, nr);

    float c = cosf(ch), s = sinf(ch);

    // For each ordered left cone, pair with its nearest right cone (robust to the
    // two walls having different visible counts), midpoint tracks the corridor.
    // Heavy smoothing below removes the per-corner skew that raw midpoints have.
    float maxg2 = LOCAL_MAX_GATE * LOCAL_MAX_GATE;
    static Mid mids[ECU_MAX_CONES];
    int nm = 0;
    for (int i = 0; i < nl; i++) {
        int best = -1;
        float bd = maxg2;
        for (int j = 0; j < nr; j++) {
            float dx = R[j].x - L[i].x, dy = R[j].y - L[i].y;
            float d2 = dx * dx + dy * dy;
            if (d2 < bd) { bd = d2; best = j; }
        }
        if (best < 0) continue;
        float mx = 0.5f * (L[i].x + R[best].x), my = 0.5f * (L[i].y + R[best].y);
        mids[nm].x   = mx;
        mids[nm].y   = my;
        mids[nm].fwd = (mx - px) * c + (my - py) * s;
        nm++;
    }

    // sort midpoints by forward distance
    for (int a = 1; a < nm; a++) {
        Mid t = mids[a];
        int b = a - 1;
        while (b >= 0 && mids[b].fwd > t.fwd) {
            mids[b + 1] = mids[b];
            b--;
        }
        mids[b + 1] = t;
    }

    // 3-point moving-average smooth to remove residual midpoint jitter
    static Mid sm[ECU_MAX_CONES];
    for (int a = 0; a < nm; a++) {
        int lo = a > 0 ? a - 1 : a, hi = a < nm - 1 ? a + 1 : a;
        sm[a].x   = (mids[lo].x + mids[a].x + mids[hi].x) / 3.0f;
        sm[a].y   = (mids[lo].y + mids[a].y + mids[hi].y) / 3.0f;
        sm[a].fwd = mids[a].fwd;
    }

    // anchor at the car, then take strictly forward-advancing, non-duplicate mids
    int count          = 0;
    m->points[count].x = px;
    m->points[count].y = py;
    count++;
    float last_fwd = 0.0f;
    for (int a = 0; a < nm && count < ECU_MAX_WAYPOINTS; a++) {
        if (sm[a].fwd < last_fwd + 0.5f) continue; // advance forward by >=0.5 m
        float dx = sm[a].x - m->points[count - 1].x;
        float dy = sm[a].y - m->points[count - 1].y;
        if (dx * dx + dy * dy < 0.25f) continue;
        m->points[count].x = sm[a].x;
        m->points[count].y = sm[a].y;
        count++;
        last_fwd = sm[a].fwd;
    }

    m->count         = count;
    m->current_index = 0;
    return count;
}

/* ---- chain-sort for the phase-2 handoff ---- */

// Reorder cones into track traversal order by a nearest-neighbour walk from the
// cone closest to the start pose. path_plan/extract_gates assume track order.
static void chain_sort(MapPoint *arr, int n, float sx, float sy)
{
    if (n < 3) return;
    // start from the cone nearest the start pose
    int first    = 0;
    float bestd  = 1e18f;
    for (int i = 0; i < n; i++) {
        float dx = arr[i].x - sx, dy = arr[i].y - sy;
        float d2 = dx * dx + dy * dy;
        if (d2 < bestd) {
            bestd = d2;
            first = i;
        }
    }
    MapPoint tmp = arr[0];
    arr[0]       = arr[first];
    arr[first]   = tmp;

    for (int i = 1; i < n; i++) {
        int best   = i;
        float bd   = 1e18f;
        for (int j = i; j < n; j++) {
            float dx = arr[j].x - arr[i - 1].x, dy = arr[j].y - arr[i - 1].y;
            float d2 = dx * dx + dy * dy;
            if (d2 < bd) {
                bd   = d2;
                best = j;
            }
        }
        MapPoint t = arr[i];
        arr[i]     = arr[best];
        arr[best]  = t;
    }
}

int online_planner_lookahead(const EcuMap *map, float px, float py, float Ld, float *tx, float *ty)
{
    if (map->count < 2) return 0;
    // Walk the polyline from the start, accumulating arc length, and return the
    // first point at least Ld away; fall back to the last point. Smooth because
    // it integrates over the whole local line, not one jittery segment.
    float acc = 0.0f;
    for (int i = 1; i < map->count; i++) {
        float dx = map->points[i].x - map->points[i - 1].x;
        float dy = map->points[i].y - map->points[i - 1].y;
        acc += sqrtf(dx * dx + dy * dy);
        if (acc >= Ld) {
            *tx = map->points[i].x;
            *ty = map->points[i].y;
            return 1;
        }
    }
    *tx = map->points[map->count - 1].x;
    *ty = map->points[map->count - 1].y;
    // only valid if the furthest point is actually ahead of the car
    float dx = *tx - px, dy = *ty - py;
    return (dx * dx + dy * dy) > 1.0f;
}

float online_planner_step(OnlinePlanner *p, const SlamState *slam, float est_x, float est_y,
    float est_heading, EcuMap *out_map)
{
    // Phase 2: serve the racing line planned once at loop closure. Do NOT
    // re-export the SLAM map here (that would wipe the planned waypoints).
    if (p->phase2_active) {
        *out_map = p->racing_line;
        return BIG_SPEED;
    }

    // Export the map every tick. Use a modest sighting bar (2): enough to admit
    // frontier cones the car must steer toward, while filtering one-off spurious
    // detections that would jitter the line.
    slam_export_cones_min(slam, out_map, 2);

    if (slam_loop_closed(slam)) {
        // Loop closed: order the confident full map and optimise a racing line once,
        // into the persistent racing_line buffer.
        slam_export_cones_min(slam, &p->racing_line, g_SLAM_MIN_SIGHTINGS);
        chain_sort(p->racing_line.left_cones, p->racing_line.left_count, est_x, est_y);
        chain_sort(p->racing_line.right_cones, p->racing_line.right_count, est_x, est_y);
        path_plan(&p->racing_line);
        if (p->racing_line.count >= 5) {
            motion_control_reset(); // clear the phase-1 progress index for the new line
            p->phase2_active = 1;
            *out_map         = p->racing_line;
            return BIG_SPEED;
        }
    }

    // Phase 1: reactive local centreline, capped speed.
    int np = build_local_centreline(out_map, est_x, est_y, est_heading);
    if (np < 2) {
        // No usable cones ahead yet: hold a straight stub so control stays sane.
        out_map->points[0].x = est_x;
        out_map->points[0].y = est_y;
        out_map->points[1].x = est_x + cosf(est_heading);
        out_map->points[1].y = est_y + sinf(est_heading);
        out_map->count       = 2;
        out_map->current_index = 0;
    }
    (void)wrap_pi;
    return g_PHASE1_SPEED_CAP_MS;
}
