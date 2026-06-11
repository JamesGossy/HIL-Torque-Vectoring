// EKF-SLAM core. Predict from odometry, update per cone observation with
// colour-gated nearest-landmark Mahalanobis association. Full dense covariance,
// fixed cap; the measurement Jacobian is nonzero only in the 3 pose columns and
// the observed landmark's 2 columns, so the gain is computed over just those.

#include "../include/slam.h"
#include "../../shared/linalg.h"
#include "../../shared/tunables.h"
#include <math.h>
#include <string.h>

#define DIM SLAM_MAX_DIM

static float P_at(const SlamState *s, int r, int c)
{
    return s->P[r * DIM + c];
}

void slam_init(SlamState *s, float start_x, float start_y, float start_heading)
{
    memset(s, 0, sizeof(*s));
    s->dim       = SLAM_POSE_DIM;
    s->n_land    = 0;
    s->mu[0]     = start_x;
    s->mu[1]     = start_y;
    s->mu[2]     = start_heading;
    s->start_x   = start_x;
    s->start_y   = start_y;
    // start pose is known exactly, so the pose covariance starts at zero
}

void slam_get_pose(const SlamState *s, float *x, float *y, float *heading)
{
    if (x) *x = s->mu[0];
    if (y) *y = s->mu[1];
    if (heading) *heading = s->mu[2];
}

int slam_loop_closed(const SlamState *s)
{
    return s->loop_closed;
}

/* ---- predict ---- */

void slam_predict(SlamState *s, const SensorData *sensors, float dt)
{
    int n     = s->dim;
    float th  = s->mu[2];
    float v   = sensors->velocity;
    float w   = sensors->yaw_rate;
    float c   = cosf(th);
    float sn  = sinf(th);

    // 1. mean: unicycle integration on the pose block
    s->mu[0] += v * c * dt;
    s->mu[1] += v * sn * dt;
    s->mu[2] = wrap_angle(th + w * dt);

    // 2. covariance: only the pose rows/cols and pose-landmark cross blocks change.
    // F = d(pose')/d(pose) is identity except F02 = -v*sin*dt, F12 = v*cos*dt.
    float f02 = -v * sn * dt;
    float f12 = v * c * dt;

    // Update the 3x3 pose block: Ppp' = F Ppp F^T + Q.
    float p00 = P_at(s, 0, 0), p01 = P_at(s, 0, 1), p02 = P_at(s, 0, 2);
    float p11 = P_at(s, 1, 1), p12 = P_at(s, 1, 2), p22 = P_at(s, 2, 2);

    // F * Ppp (row 0 and 1 pick up the theta row via f02/f12; row 2 unchanged)
    float a00 = p00 + f02 * p02, a01 = p01 + f02 * p12, a02 = p02 + f02 * p22;
    float a10 = p01 + f12 * p02, a11 = p11 + f12 * p12, a12 = p12 + f12 * p22;
    float a20 = p02, a21 = p12, a22 = p22;

    // (F Ppp) F^T
    float n00 = a00 + a02 * f02, n01 = a01 + a02 * f12, n02 = a02;
    float n10 = a10 + a12 * f02, n11 = a11 + a12 * f12, n12 = a12;
    float n20 = a20 + a22 * f02, n21 = a21 + a22 * f12, n22 = a22;

    s->P[0 * DIM + 0] = n00 + g_SLAM_Q_POS;
    s->P[0 * DIM + 1] = n01;
    s->P[0 * DIM + 2] = n02;
    s->P[1 * DIM + 0] = n10;
    s->P[1 * DIM + 1] = n11 + g_SLAM_Q_POS;
    s->P[1 * DIM + 2] = n12;
    s->P[2 * DIM + 0] = n20;
    s->P[2 * DIM + 1] = n21;
    s->P[2 * DIM + 2] = n22 + g_SLAM_Q_THETA;

    // Pose-landmark cross blocks: column j of the pose rows becomes F * (pose,j).
    for (int j = SLAM_POSE_DIM; j < n; j++) {
        float c0 = P_at(s, 0, j), c1 = P_at(s, 1, j), c2 = P_at(s, 2, j);
        float r0 = c0 + f02 * c2;
        float r1 = c1 + f12 * c2;
        float r2 = c2;
        s->P[0 * DIM + j] = r0;
        s->P[1 * DIM + j] = r1;
        s->P[2 * DIM + j] = r2;
        s->P[j * DIM + 0] = r0; // keep symmetric
        s->P[j * DIM + 1] = r1;
        s->P[j * DIM + 2] = r2;
    }
}

/* ---- update ---- */

// Predicted observation and 2x5 Jacobian over [px,py,theta,lx,ly] for landmark at slot.
static void observation_model(
    const SlamState *s, int slot, float *zr, float *zb, float H[10])
{
    float px = s->mu[0], py = s->mu[1], th = s->mu[2];
    float lx = s->mu[slot], ly = s->mu[slot + 1];
    float dx = lx - px, dy = ly - py;
    float q  = dx * dx + dy * dy;
    float r  = sqrtf(q);
    if (r < 1e-4f) r = 1e-4f;

    *zr = r;
    *zb = wrap_angle(atan2f(dy, dx) - th);

    // d range / d[px,py,theta,lx,ly]
    H[0] = -dx / r;
    H[1] = -dy / r;
    H[2] = 0.0f;
    H[3] = dx / r;
    H[4] = dy / r;
    // d bearing / d[px,py,theta,lx,ly]
    H[5] = dy / q;
    H[6] = -dx / q;
    H[7] = -1.0f;
    H[8] = -dy / q;
    H[9] = dx / q;
}

// Fold one matched observation (range,bearing) for landmark `li` into the filter.
static void apply_update(SlamState *s, int li, float zr_meas, float zb_meas)
{
    int n    = s->dim;
    int slot = s->land[li].slot;
    int idx[5] = { 0, 1, 2, slot, slot + 1 }; // the 5 active state columns

    float zr_hat, zb_hat, H[10];
    observation_model(s, slot, &zr_hat, &zb_hat, H);

    // innovation
    float y0 = zr_meas - zr_hat;
    float y1 = wrap_angle(zb_meas - zb_hat);

    float R0 = g_SENSOR_RANGE_SIGMA_M * g_SENSOR_RANGE_SIGMA_M;
    float R1 = g_SENSOR_BEARING_SIGMA_RAD * g_SENSOR_BEARING_SIGMA_RAD;

    // PHt: (n x 2). Column for output row m is sum over active cols k of P[i][idx[k]]*H[m][k].
    // Only the 5 active columns of H are nonzero, so this is cheap.
    static float PHt[DIM * 2];
    for (int i = 0; i < n; i++) {
        float s0 = 0.0f, s1 = 0.0f;
        for (int k = 0; k < 5; k++) {
            float pik = s->P[i * DIM + idx[k]];
            s0 += pik * H[k];     // range row
            s1 += pik * H[5 + k]; // bearing row
        }
        PHt[i * 2 + 0] = s0;
        PHt[i * 2 + 1] = s1;
    }

    // S = H * PHt + R, 2x2. H picks the active rows of PHt.
    float S[4] = { R0, 0.0f, 0.0f, R1 };
    for (int k = 0; k < 5; k++) {
        int c = idx[k];
        S[0] += H[k] * PHt[c * 2 + 0];
        S[1] += H[k] * PHt[c * 2 + 1];
        S[2] += H[5 + k] * PHt[c * 2 + 0];
        S[3] += H[5 + k] * PHt[c * 2 + 1];
    }

    float Sinv[4];
    if (!inv2x2(S, Sinv)) return; // singular, skip rather than diverge

    // K = PHt * Sinv, (n x 2). Store it; the Joseph update reuses it.
    static float K[DIM * 2];
    for (int i = 0; i < n; i++) {
        K[i * 2 + 0] = PHt[i * 2 + 0] * Sinv[0] + PHt[i * 2 + 1] * Sinv[2];
        K[i * 2 + 1] = PHt[i * 2 + 0] * Sinv[1] + PHt[i * 2 + 1] * Sinv[3];
    }

    // mean: mu += K y
    for (int i = 0; i < n; i++)
        s->mu[i] += K[i * 2 + 0] * y0 + K[i * 2 + 1] * y1;
    s->mu[2] = wrap_angle(s->mu[2]);

    // Covariance, Joseph form: P' = (I-KH) P (I-KH)^T + K R K^T. Numerically
    // stable (stays symmetric positive-definite) where the plain P -= K H P form
    // drifts non-PD under aggressive maneuvers and diverges. H has only the 5
    // active columns idx[], so KH touches just those columns.
    // 1. A = (I - K H) P  (n x n). (K H P)[i][j] = sum_m K[i][m] (H P)[m][j],
    //    and (H P)[m][j] = PHt[j][m]. So A[i][j] = P[i][j] - K[i].PHt[j].
    static float A[DIM * DIM];
    for (int i = 0; i < n; i++) {
        float ki0 = K[i * 2 + 0], ki1 = K[i * 2 + 1];
        const float *Prow = &s->P[i * DIM];
        float *Arow       = &A[i * DIM];
        for (int j = 0; j < n; j++)
            Arow[j] = Prow[j] - (ki0 * PHt[j * 2 + 0] + ki1 * PHt[j * 2 + 1]);
    }
    // 2. AHt = A H^T (n x 2), using the 5 active columns of H.
    static float AHt[DIM * 2];
    for (int i = 0; i < n; i++) {
        float a0 = 0.0f, a1 = 0.0f;
        const float *Arow = &A[i * DIM];
        for (int k = 0; k < 5; k++) {
            float aik = Arow[idx[k]];
            a0 += aik * H[k];
            a1 += aik * H[5 + k];
        }
        AHt[i * 2 + 0] = a0;
        AHt[i * 2 + 1] = a1;
    }
    // 3. P' = A - AHt K^T + K R K^T, with R diagonal (R0,R1).
    for (int i = 0; i < n; i++) {
        float ki0 = K[i * 2 + 0], ki1 = K[i * 2 + 1];
        float ai0 = AHt[i * 2 + 0], ai1 = AHt[i * 2 + 1];
        const float *Arow = &A[i * DIM];
        float *Prow       = &s->P[i * DIM];
        for (int j = 0; j < n; j++) {
            float kj0 = K[j * 2 + 0], kj1 = K[j * 2 + 1];
            float krk = ki0 * R0 * kj0 + ki1 * R1 * kj1;
            float ahk = ai0 * kj0 + ai1 * kj1; // (AHt K^T)[i][j]
            Prow[j]   = Arow[j] - ahk + krk;
        }
    }
}

// Initialise a new landmark from an observation via the inverse measurement model.
static int init_landmark(SlamState *s, float range, float bearing, int color)
{
    if (s->n_land >= SLAM_MAX_LANDMARKS) return -1;
    int slot = s->dim;
    if (slot + 2 > SLAM_MAX_DIM) return -1;

    int n  = s->dim;
    float px = s->mu[0], py = s->mu[1], th = s->mu[2];
    float ca = cosf(th + bearing), sa = sinf(th + bearing);
    s->mu[slot]     = px + range * ca;
    s->mu[slot + 1] = py + range * sa;

    // Inverse-observation Jacobians. The pose Jacobian Gp couples the new landmark
    // to the pose so that later sightings correct BOTH; a diagonal seed would
    // leave the pose uncorrectable and the whole map drifts with odometry.
    // Gp (2x3) = d l / d[px,py,theta]; Gz (2x2) = d l / d[range,bearing].
    float gp[6] = { 1.0f, 0.0f, -range * sa, 0.0f, 1.0f, range * ca };
    float gz[4] = { ca, -range * sa, sa, range * ca };
    float R0    = g_SENSOR_RANGE_SIGMA_M * g_SENSOR_RANGE_SIGMA_M;
    float R1    = g_SENSOR_BEARING_SIGMA_RAD * g_SENSOR_BEARING_SIGMA_RAD;

    // Cross-covariance to every existing state column j: P_Lj = Gp * P[pose,j].
    for (int j = 0; j < n; j++) {
        float c0 = s->P[0 * DIM + j], c1 = s->P[1 * DIM + j], c2 = s->P[2 * DIM + j];
        float l0 = gp[0] * c0 + gp[1] * c1 + gp[2] * c2;
        float l1 = gp[3] * c0 + gp[4] * c1 + gp[5] * c2;
        s->P[slot * DIM + j]       = l0;
        s->P[(slot + 1) * DIM + j] = l1;
        s->P[j * DIM + slot]       = l0;
        s->P[j * DIM + slot + 1]   = l1;
    }

    // P_LL = Gp Ppp Gp^T + Gz R Gz^T.
    float ppp[9] = { s->P[0 * DIM + 0], s->P[0 * DIM + 1], s->P[0 * DIM + 2], s->P[1 * DIM + 0],
        s->P[1 * DIM + 1], s->P[1 * DIM + 2], s->P[2 * DIM + 0], s->P[2 * DIM + 1],
        s->P[2 * DIM + 2] };
    // GpPpp (2x3)
    float a[6];
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 3; c++)
            a[r * 3 + c] = gp[r * 3 + 0] * ppp[0 * 3 + c] + gp[r * 3 + 1] * ppp[1 * 3 + c]
                + gp[r * 3 + 2] * ppp[2 * 3 + c];
    float pll[4];
    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 2; c++)
            pll[r * 2 + c] = a[r * 3 + 0] * gp[c * 3 + 0] + a[r * 3 + 1] * gp[c * 3 + 1]
                + a[r * 3 + 2] * gp[c * 3 + 2];
    // + Gz diag(R0,R1) Gz^T
    pll[0] += gz[0] * R0 * gz[0] + gz[1] * R1 * gz[1];
    pll[1] += gz[0] * R0 * gz[2] + gz[1] * R1 * gz[3];
    pll[2] += gz[2] * R0 * gz[0] + gz[3] * R1 * gz[1];
    pll[3] += gz[2] * R0 * gz[2] + gz[3] * R1 * gz[3];

    s->P[slot * DIM + slot]           = pll[0];
    s->P[slot * DIM + slot + 1]       = pll[1];
    s->P[(slot + 1) * DIM + slot]     = pll[2];
    s->P[(slot + 1) * DIM + slot + 1] = pll[3];

    int li               = s->n_land;
    s->land[li].color    = color;
    s->land[li].seen_count = 1;
    s->land[li].slot     = slot;
    s->n_land++;
    s->dim += 2;

    if (s->n_start_land < 8) s->start_land_ids[s->n_start_land++] = li;
    return li;
}

// Best same-colour landmark by Mahalanobis distance; writes best d2 and the
// second-best d2 so the caller can reject ambiguous matches (two cones equally
// plausible -> a wrong correspondence that would drag the pose).
static int associate(
    SlamState *s, float range, float bearing, int color, float *out_d2, float *out_d2_second)
{
    float R0 = g_SENSOR_RANGE_SIGMA_M * g_SENSOR_RANGE_SIGMA_M;
    float R1 = g_SENSOR_BEARING_SIGMA_RAD * g_SENSOR_BEARING_SIGMA_RAD;
    int best = -1;
    float best_d2   = 1e18f;
    float second_d2 = 1e18f;

    for (int li = 0; li < s->n_land; li++) {
        if (s->land[li].color != color) continue;
        int slot = s->land[li].slot;
        int idx[5] = { 0, 1, 2, slot, slot + 1 };

        float zr_hat, zb_hat, H[10];
        observation_model(s, slot, &zr_hat, &zb_hat, H);
        float y0 = range - zr_hat;
        float y1 = wrap_angle(bearing - zb_hat);

        // S = H P H^T + R (2x2), using only active columns
        float S[4] = { R0, 0.0f, 0.0f, R1 };
        for (int a = 0; a < 5; a++) {
            for (int b = 0; b < 5; b++) {
                float pab = s->P[idx[a] * DIM + idx[b]];
                S[0] += H[a] * pab * H[b];
                S[1] += H[a] * pab * H[5 + b];
                S[2] += H[5 + a] * pab * H[b];
                S[3] += H[5 + a] * pab * H[5 + b];
            }
        }
        float Sinv[4];
        if (!inv2x2(S, Sinv)) continue;
        float d2 = y0 * (Sinv[0] * y0 + Sinv[1] * y1) + y1 * (Sinv[2] * y0 + Sinv[3] * y1);
        if (d2 < best_d2) {
            second_d2 = best_d2;
            best_d2   = d2;
            best      = li;
        } else if (d2 < second_d2) {
            second_d2 = d2;
        }
    }
    if (out_d2) *out_d2 = best_d2;
    if (out_d2_second) *out_d2_second = second_d2;
    return best;
}

// World distance from an observation's implied point to the nearest same-colour landmark.
static float nearest_landmark_dist(const SlamState *s, float range, float bearing, int color)
{
    float th = s->mu[2];
    float wx = s->mu[0] + range * cosf(th + bearing);
    float wy = s->mu[1] + range * sinf(th + bearing);
    float best = 1e18f;
    for (int li = 0; li < s->n_land; li++) {
        if (s->land[li].color != color) continue;
        int slot = s->land[li].slot;
        float dx = s->mu[slot] - wx, dy = s->mu[slot + 1] - wy;
        float d  = sqrtf(dx * dx + dy * dy);
        if (d < best) best = d;
    }
    return best;
}

void slam_update(SlamState *s, const ConeScan *scan)
{
    for (int i = 0; i < scan->count; i++) {
        float range   = scan->obs[i].range;
        float bearing = scan->obs[i].bearing;
        int color     = scan->obs[i].color;

        float d2 = 1e18f, d2_second = 1e18f;
        int li   = associate(s, range, bearing, color, &d2, &d2_second);

        // Accept only an unambiguous match: inside the gate AND clearly closer
        // than the runner-up. With densely spaced cones plus noise the runner-up
        // is often a neighbour, and a confident update against the wrong cone
        // drags the pose; rejecting ambiguous matches is what keeps fusion from
        // degrading a good estimate through corners.
        int unambiguous = (d2_second > g_SLAM_AMBIG_RATIO * d2);

        if (li >= 0 && d2 < g_SLAM_GATE_CHI2 && unambiguous) {
            s->land[li].seen_count++;
            apply_update(s, li, range, bearing);

            // loop closure: re-seeing a start landmark after leaving the start
            if (s->moved_away && !s->loop_closed) {
                for (int k = 0; k < s->n_start_land; k++) {
                    if (s->start_land_ids[k] == li) {
                        float dx = s->mu[0] - s->start_x, dy = s->mu[1] - s->start_y;
                        if (dx * dx + dy * dy < g_SLAM_LOOP_RADIUS_M * g_SLAM_LOOP_RADIUS_M)
                            s->loop_closed = 1;
                    }
                }
            }
        } else if (d2 > g_SLAM_NEW_CHI2
            && nearest_landmark_dist(s, range, bearing, color) > g_SLAM_NEW_MIN_DIST_M) {
            init_landmark(s, range, bearing, color);
        }
    }

    // track whether we have left the start neighbourhood (gate for loop closure)
    float dx = s->mu[0] - s->start_x, dy = s->mu[1] - s->start_y;
    if (dx * dx + dy * dy > 15.0f * 15.0f) s->moved_away = 1;
}

/* ---- export ---- */

void slam_export_cones_min(const SlamState *s, EcuMap *out, int min_sightings)
{
    out->count         = 0;
    out->current_index = 0;
    out->left_count    = 0;
    out->right_count   = 0;
    for (int li = 0; li < s->n_land; li++) {
        if (s->land[li].seen_count < min_sightings) continue;
        int slot = s->land[li].slot;
        float x  = s->mu[slot], y = s->mu[slot + 1];
        if (s->land[li].color == CONE_COLOR_LEFT) {
            if (out->left_count < ECU_MAX_CONES) {
                out->left_cones[out->left_count].x = x;
                out->left_cones[out->left_count].y = y;
                out->left_count++;
            }
        } else {
            if (out->right_count < ECU_MAX_CONES) {
                out->right_cones[out->right_count].x = x;
                out->right_cones[out->right_count].y = y;
                out->right_count++;
            }
        }
    }
}

void slam_export_cones(const SlamState *s, EcuMap *out)
{
    slam_export_cones_min(s, out, g_SLAM_MIN_SIGHTINGS);
}
