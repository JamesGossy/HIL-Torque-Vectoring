/* Unit tests for path_plan(). Build and run via: make test */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "../HIL_Firmware/include/path_planning.h"
#include "../HIL_Firmware/include/track_parser.h"

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

/* Build an oval track: n cones per side forming a rectangle, evenly spaced along x. */
static Track make_oval(int n, float length, float half_w)
{
    Track t;
    memset(&t, 0, sizeof(t));
    if (n > MAX_CONES) n = MAX_CONES;
    for (int i = 0; i < n; i++) {
        float x            = length * i / (float)(n - 1);
        t.left_cones[i].x  = x;
        t.left_cones[i].y  = half_w;
        t.right_cones[i].x = x;
        t.right_cones[i].y = -half_w;
    }
    t.left_count  = n;
    t.right_count = n;
    return t;
}

/* path_plan must produce at least a few waypoints from a valid cone set */
static void test_produces_waypoints(void)
{
    Track t = make_oval(20, 40.0f, 2.0f);
    path_plan(&t);
    ASSERT(t.count >= 4);
}

/* waypoints must stay within the corridor (with a small tolerance for margin) */
static void test_waypoints_inside_corridor(void)
{
    float half_w = 2.5f;
    Track t      = make_oval(30, 60.0f, half_w);
    path_plan(&t);

    for (int i = 0; i < t.count; i++)
        ASSERT(fabsf(t.points[i].y) <= half_w + 0.1f);
}

/* all waypoints must have finite coordinates */
static void test_no_nan_or_inf(void)
{
    Track t = make_oval(25, 50.0f, 2.0f);
    path_plan(&t);

    for (int i = 0; i < t.count; i++) {
        ASSERT(isfinite(t.points[i].x));
        ASSERT(isfinite(t.points[i].y));
    }
}

/* a wider track gives waypoints at roughly the centreline on a straight */
static void test_centreline_on_straight(void)
{
    Track t = make_oval(20, 40.0f, 3.0f);
    path_plan(&t);

    float sum_y = 0.0f; // mean y should be near 0 on a symmetric track
    for (int i = 0; i < t.count; i++)
        sum_y += t.points[i].y;
    float mean_y = sum_y / (float)t.count;
    ASSERT_NEAR(mean_y, 0.0f, 0.5f);
}

/* more cones should not crash or produce zero waypoints */
static void test_large_cone_count(void)
{
    Track t = make_oval(MAX_CONES, 100.0f, 2.0f);
    path_plan(&t);
    ASSERT(t.count > 0);
    ASSERT(t.count <= MAX_WAYPOINTS);
}

/* a minimal cone set (2 per side) must not crash */
static void test_minimal_cone_set(void)
{
    Track t;
    memset(&t, 0, sizeof(t));
    t.left_cones[0].x  = 0.0f;
    t.left_cones[0].y  = 2.0f;
    t.left_cones[1].x  = 5.0f;
    t.left_cones[1].y  = 2.0f;
    t.right_cones[0].x = 0.0f;
    t.right_cones[0].y = -2.0f;
    t.right_cones[1].x = 5.0f;
    t.right_cones[1].y = -2.0f;
    t.left_count       = 2;
    t.right_count      = 2;
    path_plan(&t);
    ASSERT(t.count >= 0);
}

/* calling path_plan twice must give the same result (deterministic) */
static void test_deterministic(void)
{
    Track t1 = make_oval(20, 40.0f, 2.0f);
    Track t2 = t1;

    path_plan(&t1);
    path_plan(&t2);

    ASSERT(t1.count == t2.count);
    for (int i = 0; i < t1.count; i++) {
        ASSERT_NEAR(t1.points[i].x, t2.points[i].x, 1e-4f);
        ASSERT_NEAR(t1.points[i].y, t2.points[i].y, 1e-4f);
    }
}

int main(void)
{
    test_produces_waypoints();
    test_waypoints_inside_corridor();
    test_no_nan_or_inf();
    test_centreline_on_straight();
    test_large_cone_count();
    test_minimal_cone_set();
    test_deterministic();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
