/*
 * tests/test_steer.c
 *
 * Unit tests for the steering law (HIL_Firmware/src/motion_control.c
 * steer_command): a kinematic curvature feedforward plus Stanley feedback that
 * replaced the old model-based LQR.
 *
 * Build and run via:   make test   (from the repo root)
 *
 * These pin the controller's contract - finiteness and boundedness across the
 * speed range, the equilibrium (zero error, straight path -> ~zero command), and
 * the correcting signs of the cross-track feedback, the heading feedback, and the
 * curvature feedforward. They do NOT re-derive the lap result (that is `make
 * eval`); they catch the sign/scaling mistakes a refactor is most likely to make.
 */

#include <math.h>
#include <stdio.h>

#include "../HIL_Firmware/include/motion_control.h"

/* ---- Minimal test framework ---- */

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

static int is_finite_f(float x)
{
    return x == x && fabsf(x) < 1e30f;
}

/* ---- Tests ---- */

/* The command must be finite and bounded across the whole speed range, including
 * the low-speed regime where the atan denominator is small. */
static void test_command_finite_across_speeds(void)
{
    for (float vx = 0.0f; vx <= 35.0f; vx += 0.5f) {
        float u = steer_command(vx, 0.1f, 0.05f, 0.0f, 0.02f);
        ASSERT(is_finite_f(u));
        ASSERT(fabsf(u) < 50.0f); /* reference units, but must stay sane */
    }
}

/* On the line, aligned, on a straight path: no error to feed back and no
 * curvature to feed forward, so the command is ~zero. */
static void test_zero_error_zero_command(void)
{
    float u = steer_command(20.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    ASSERT_NEAR(u, 0.0f, 1e-3f);
}

/* Cross-track sign: e1 = -cte, so +e1 means the car is LEFT of the line and must
 * steer RIGHT. Positive reference is a left turn, so a right correction is a
 * NEGATIVE command. */
static void test_cross_track_sign(void)
{
    float u_left  = steer_command(20.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    float u_right = steer_command(20.0f, -0.3f, 0.0f, 0.0f, 0.0f);
    ASSERT(u_left < 0.0f);                /* car left of line  -> steer right (negative) */
    ASSERT(u_right > 0.0f);               /* car right of line -> steer left  (positive) */
    ASSERT_NEAR(u_left, -u_right, 1e-4f); /* symmetric about the line */
}

/* Heading-error sign: e2 = heading - path, so +e2 (nose pointing left of the
 * path tangent) must produce a right (negative) correction. */
static void test_heading_error_sign(void)
{
    float u = steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f);
    ASSERT(u < 0.0f);
}

/* Curvature feedforward sign: +path_kappa is a left-hand bend, which needs a
 * positive (left) reference even with zero tracking error, and is symmetric. */
static void test_curvature_feedforward_sign(void)
{
    /* Pass yaw_rate = vx*kappa so the damping term is zero and this isolates the
     * feedforward sign. */
    float u_left  = steer_command(20.0f, 0.0f, 0.0f, 20.0f * 0.05f, 0.05f);
    float u_right = steer_command(20.0f, 0.0f, 0.0f, 20.0f * -0.05f, -0.05f);
    ASSERT(u_left > 0.0f);
    ASSERT(u_right < 0.0f);
    ASSERT_NEAR(u_left, -u_right, 1e-3f);
}

/* Yaw-rate damping sign: with no tracking error and a straight path (kappa=0),
 * a car that is over-rotating (positive yaw rate, turning left) must get a right
 * (negative) correction to damp it. */
static void test_yaw_damping_sign(void)
{
    float u = steer_command(20.0f, 0.0f, 0.0f, 0.2f, 0.0f); /* +yaw, straight path */
    ASSERT(u < 0.0f);
    /* symmetric: opposite yaw rate -> opposite correction */
    float u2 = steer_command(20.0f, 0.0f, 0.0f, -0.2f, 0.0f);
    ASSERT(u2 > 0.0f);
    ASSERT_NEAR(u, -u2, 1e-4f);
}

/* The derived understeer gradient must be a finite, sane number. */
static void test_understeer_gradient_finite(void)
{
    float k = driver_understeer_gradient();
    ASSERT(is_finite_f(k));
    ASSERT(fabsf(k) < 1.0f);
}

/* ---- Entry point ---- */

int main(void)
{
    test_command_finite_across_speeds();
    test_zero_error_zero_command();
    test_cross_track_sign();
    test_heading_error_sign();
    test_curvature_feedforward_sign();
    test_yaw_damping_sign();
    test_understeer_gradient_finite();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
