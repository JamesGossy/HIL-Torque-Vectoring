/*
 * Unit tests for the steering law (steer_command in motion_control.c): a
 * kinematic curvature feedforward plus Stanley feedback. They pin finiteness,
 * boundedness, the zero-error equilibrium, and the correcting signs. They do
 * not re-derive the lap result, that is make eval.
 */

#include <math.h>
#include <stdio.h>

#include "../ECU_Firmware/include/motion_control.h"

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

// Command stays finite and bounded across the whole speed range.
static void test_command_finite_across_speeds(void)
{
    for (float vx = 0.0f; vx <= 35.0f; vx += 0.5f) {
        float u = steer_command(vx, 0.1f, 0.05f, 0.0f, 0.02f);
        ASSERT(is_finite_f(u));
        ASSERT(fabsf(u) < 50.0f);
    }
}

// Zero error on a straight path gives a near-zero command.
static void test_zero_error_zero_command(void)
{
    float u = steer_command(20.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    ASSERT_NEAR(u, 0.0f, 1e-3f);
}

// Cross-track sign and symmetry about the line.
static void test_cross_track_sign(void)
{
    float u_left  = steer_command(20.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    float u_right = steer_command(20.0f, -0.3f, 0.0f, 0.0f, 0.0f);
    ASSERT(u_left < 0.0f);                // car left of line, steer right (negative)
    ASSERT(u_right > 0.0f);               // car right of line, steer left (positive)
    ASSERT_NEAR(u_left, -u_right, 1e-4f); // symmetric about the line
}

// Heading-error sign: +e2 must produce a right (negative) correction.
static void test_heading_error_sign(void)
{
    float u = steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f);
    ASSERT(u < 0.0f);
}

// Curvature feedforward sign and symmetry.
static void test_curvature_feedforward_sign(void)
{
    // yaw_rate = vx*kappa zeroes the damping term to isolate the feedforward sign
    float u_left  = steer_command(20.0f, 0.0f, 0.0f, 20.0f * 0.05f, 0.05f);
    float u_right = steer_command(20.0f, 0.0f, 0.0f, 20.0f * -0.05f, -0.05f);
    ASSERT(u_left > 0.0f);
    ASSERT(u_right < 0.0f);
    ASSERT_NEAR(u_left, -u_right, 1e-3f);
}

// Yaw-rate damping sign and symmetry.
static void test_yaw_damping_sign(void)
{
    float u = steer_command(20.0f, 0.0f, 0.0f, 0.2f, 0.0f); // +yaw, straight path
    ASSERT(u < 0.0f);
    float u2 = steer_command(20.0f, 0.0f, 0.0f, -0.2f, 0.0f);
    ASSERT(u2 > 0.0f);
    ASSERT_NEAR(u, -u2, 1e-4f);
}

// The derived understeer gradient must be a finite, sane number.
static void test_understeer_gradient_finite(void)
{
    float k = driver_understeer_gradient();
    ASSERT(is_finite_f(k));
    ASSERT(fabsf(k) < 1.0f);
}

// Runs every steering test and reports the pass count.
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
