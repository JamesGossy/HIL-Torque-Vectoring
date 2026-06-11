/* Unit tests for the EKF-SLAM core and its linear algebra. Build via make test. */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../shared/linalg.h"
#include "../ECU_Firmware/include/slam.h"
#include "../shared/tunables.h"

static int g_tests  = 0;
static int g_passed = 0;

#define ASSERT(cond)                                                                               \
    do {                                                                                           \
        g_tests++;                                                                                 \
        if (cond) {                                                                                \
            g_passed++;                                                                            \
        } else {                                                                                   \
            fprintf(stderr, "FAIL  %s:%d  (%s)\n", __FILE__, __LINE__, #cond);                     \
        }                                                                                          \
    } while (0)

#define ASSERT_NEAR(a, b, tol) ASSERT(fabsf((float)(a) - (float)(b)) <= (float)(tol))

/* ---- linalg ---- */

static void test_inv2x2(void)
{
    float A[4] = { 4.0f, 7.0f, 2.0f, 6.0f };
    float Ai[4], I[4];
    ASSERT(inv2x2(A, Ai) == 1);
    mat_mul(A, 2, 2, Ai, 2, I);
    ASSERT_NEAR(I[0], 1.0f, 1e-4f);
    ASSERT_NEAR(I[1], 0.0f, 1e-4f);
    ASSERT_NEAR(I[2], 0.0f, 1e-4f);
    ASSERT_NEAR(I[3], 1.0f, 1e-4f);

    float Sing[4] = { 1.0f, 2.0f, 2.0f, 4.0f };
    float So[4];
    ASSERT(inv2x2(Sing, So) == 0);
}

static void test_wrap_angle(void)
{
    ASSERT_NEAR(wrap_angle(0.0f), 0.0f, 1e-6f);
    ASSERT_NEAR(wrap_angle(3.0f * LINALG_PI), LINALG_PI, 1e-4f);
    ASSERT(fabsf(fabsf(wrap_angle(-3.0f * LINALG_PI)) - LINALG_PI) <= 1e-4f); // +/-pi both valid
}

/* ---- a noise-free synthetic detector for the filter tests ---- */

static float wrap_pi(float a)
{
    while (a > LINALG_PI)
        a -= 2.0f * LINALG_PI;
    while (a < -LINALG_PI)
        a += 2.0f * LINALG_PI;
    return a;
}

/* Build a perfect (noise-free) scan of the given cones from a true pose. */
static void perfect_scan(ConeScan *scan, const float (*cones)[3], int n, float x, float y, float th)
{
    scan->count = 0;
    for (int i = 0; i < n && scan->count < MAX_OBS_PER_TICK; i++) {
        float dx              = cones[i][0] - x, dy = cones[i][1] - y;
        scan->obs[scan->count].range   = sqrtf(dx * dx + dy * dy);
        scan->obs[scan->count].bearing = wrap_pi(atan2f(dy, dx) - th);
        scan->obs[scan->count].color   = (int)cones[i][2];
        scan->count++;
    }
}

static float pose_trace(const SlamState *s)
{
    return s->P[0] + s->P[1 * SLAM_MAX_DIM + 1] + s->P[2 * SLAM_MAX_DIM + 2];
}

/* Predict with no update must grow the pose covariance trace. */
static void test_predict_grows_trace(void)
{
    tunables_init_from_env();
    SlamState s;
    slam_init(&s, 0.0f, 0.0f, 0.0f);

    SensorData sd = { 0 };
    sd.velocity   = 5.0f;
    sd.yaw_rate   = 0.1f;

    float t0 = pose_trace(&s);
    for (int i = 0; i < 50; i++)
        slam_predict(&s, &sd, 0.01f);
    float t1 = pose_trace(&s);
    ASSERT(t1 > t0);
}

/* Repeated consistent observations of a known landmark must converge its estimate. */
static void test_update_converges_landmark(void)
{
    tunables_init_from_env();
    SlamState s;
    slam_init(&s, 0.0f, 0.0f, 0.0f);

    /* one cone 5 m ahead and 1 m left */
    float cones[1][3] = { { 5.0f, 1.0f, (float)CONE_COLOR_LEFT } };

    SensorData sd = { 0 }; /* stationary: pose stays at origin */
    ConeScan scan;

    for (int i = 0; i < 60; i++) {
        slam_predict(&s, &sd, 0.01f);
        perfect_scan(&scan, cones, 1, s.mu[0], s.mu[1], s.mu[2]);
        slam_update(&s, &scan);
    }

    ASSERT(s.n_land == 1); /* exactly one landmark, no duplicates */
    int slot = s.land[0].slot;
    ASSERT_NEAR(s.mu[slot], 5.0f, 0.2f);
    ASSERT_NEAR(s.mu[slot + 1], 1.0f, 0.2f);
}

/* Updates against a fixed landmark must shrink its covariance trace over time. */
static void test_update_shrinks_landmark_cov(void)
{
    tunables_init_from_env();
    SlamState s;
    slam_init(&s, 0.0f, 0.0f, 0.0f);
    float cones[1][3] = { { 6.0f, 0.0f, (float)CONE_COLOR_RIGHT } };
    SensorData sd     = { 0 };
    ConeScan scan;

    /* seed the landmark */
    perfect_scan(&scan, cones, 1, 0, 0, 0);
    slam_update(&s, &scan);
    int slot   = s.land[0].slot;
    float tr0  = s.P[slot * SLAM_MAX_DIM + slot] + s.P[(slot + 1) * SLAM_MAX_DIM + slot + 1];

    for (int i = 0; i < 30; i++) {
        slam_predict(&s, &sd, 0.01f);
        perfect_scan(&scan, cones, 1, s.mu[0], s.mu[1], s.mu[2]);
        slam_update(&s, &scan);
    }
    float tr1 = s.P[slot * SLAM_MAX_DIM + slot] + s.P[(slot + 1) * SLAM_MAX_DIM + slot + 1];
    ASSERT(tr1 < tr0);
}

/* Distinct-colour cones at the same place must not merge. */
static void test_color_gating(void)
{
    tunables_init_from_env();
    SlamState s;
    slam_init(&s, 0.0f, 0.0f, 0.0f);
    float cones[2][3] = { { 5.0f, 0.0f, (float)CONE_COLOR_LEFT },
        { 5.0f, 0.2f, (float)CONE_COLOR_RIGHT } };
    ConeScan scan;
    for (int i = 0; i < 5; i++) {
        perfect_scan(&scan, cones, 2, 0, 0, 0);
        slam_update(&s, &scan);
    }
    ASSERT(s.n_land == 2);
}

/* A stable pose seeing the same cones repeatedly must not double-count them. */
static void test_no_double_count(void)
{
    tunables_init_from_env();
    SlamState s;
    slam_init(&s, 0.0f, 0.0f, 0.0f);
    float cones[3][3] = { { 5.0f, 1.0f, (float)CONE_COLOR_LEFT },
        { 5.0f, -1.0f, (float)CONE_COLOR_RIGHT }, { 8.0f, 1.0f, (float)CONE_COLOR_LEFT } };
    SensorData sd = { 0 };
    ConeScan scan;
    for (int i = 0; i < 100; i++) {
        slam_predict(&s, &sd, 0.01f);
        perfect_scan(&scan, cones, 3, s.mu[0], s.mu[1], s.mu[2]);
        slam_update(&s, &scan);
    }
    ASSERT(s.n_land == 3);
}

int main(void)
{
    test_inv2x2();
    test_wrap_angle();
    test_predict_grows_trace();
    test_update_converges_landmark();
    test_update_shrinks_landmark_cov();
    test_color_gating();
    test_no_double_count();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
