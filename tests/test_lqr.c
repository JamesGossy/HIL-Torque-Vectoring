/*
 * tests/test_lqr.c
 *
 * Unit tests for the model-based LQR steering law (HIL_Firmware/src/lqr_steer.c).
 *
 * Build and run via:   make test   (from the repo root)
 *
 * These check the controller's contract - finiteness of the Riccati-derived
 * gain across the speed range, the equilibrium (zero error -> ~zero command),
 * the correcting signs of the feedback and the curvature feedforward, and that
 * lqr_steer_reset() actually clears the internal cross-track integrator. They do
 * NOT re-derive the lap result (that is what `make eval` is for); they pin the
 * sign conventions and numerical sanity a refactor is most likely to break.
 */

#include <stdio.h>
#include <math.h>

#include "../HIL_Firmware/include/lqr_steer.h"

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

/* The gain must be finite and the command bounded across the whole speed range,
 * including the low-speed regime where the error model is singular and the code
 * clamps vx. A NaN or blow-up here would mean the Riccati iteration diverged. */
static void test_command_finite_across_speeds(void)
{
    for (float vx = 0.0f; vx <= 35.0f; vx += 0.5f) {
        lqr_steer_reset();
        float u = lqr_steer_command(vx, 0.0f, 0.1f, 0.05f, 0.0f, 0.02f);
        ASSERT(is_finite_f(u));
        ASSERT(fabsf(u) < 50.0f); /* reference units, but must stay sane */
    }
}

/* On the line, aligned, on a straight path, the command should be ~zero: no
 * error to feed back and no curvature to feed forward. */
static void test_zero_error_zero_command(void)
{
    lqr_steer_reset();
    float u = lqr_steer_command(20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    ASSERT_NEAR(u, 0.0f, 1e-3f);
}

/* Feedback sign: e1 = -cte, so +e1 means the car is LEFT of the line and must
 * steer RIGHT. Positive steering reference is a left turn, so a right correction
 * is a NEGATIVE command. (One tick, so the integrator has not yet built up.) */
static void test_feedback_sign(void)
{
    lqr_steer_reset();
    float u_left = lqr_steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    lqr_steer_reset();
    float u_right = lqr_steer_command(20.0f, 0.0f, -0.3f, 0.0f, 0.0f, 0.0f);
    ASSERT(u_left < 0.0f);  /* car left of line -> steer right (negative) */
    ASSERT(u_right > 0.0f); /* car right of line -> steer left (positive) */
}

/* Heading-error sign: +e2 (car pointing left of the path tangent) must produce a
 * right (negative) correction to swing the nose back. */
static void test_heading_error_sign(void)
{
    lqr_steer_reset();
    float u = lqr_steer_command(20.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.0f);
    ASSERT(u < 0.0f);
}

/* Curvature feedforward sign: +path_kappa is a left-hand bend, which needs a
 * positive (left) steering reference even with zero tracking error. */
static void test_curvature_feedforward_sign(void)
{
    lqr_steer_reset();
    float u_left = lqr_steer_command(20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
    lqr_steer_reset();
    float u_right = lqr_steer_command(20.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.05f);
    ASSERT(u_left > 0.0f);
    ASSERT(u_right < 0.0f);
    /* symmetric about zero curvature */
    ASSERT_NEAR(u_left, -u_right, 1e-3f);
}

/* The integrator builds up under a sustained offset, so the command grows over
 * repeated identical calls; after a reset the very first call must return to the
 * single-tick value (i.e. reset cleared the integral state). */
static void test_reset_clears_integrator(void)
{
    lqr_steer_reset();
    float first = lqr_steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 200; i++)
        (void)lqr_steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    float wound = lqr_steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    ASSERT(fabsf(wound) > fabsf(first)); /* integral made the command grow */

    lqr_steer_reset();
    float after_reset = lqr_steer_command(20.0f, 0.0f, 0.3f, 0.0f, 0.0f, 0.0f);
    ASSERT_NEAR(after_reset, first, 1e-4f);
}

/* The cross-track integral contribution is clamped, so even a very long
 * sustained offset cannot drive the command unbounded. */
static void test_integrator_clamped(void)
{
    lqr_steer_reset();
    float u = 0.0f;
    for (int i = 0; i < 5000; i++)
        u = lqr_steer_command(20.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f);
    ASSERT(is_finite_f(u));
    ASSERT(fabsf(u) < 50.0f);
}

/* ---- Entry point ---- */

int main(void)
{
    test_command_finite_across_speeds();
    test_zero_error_zero_command();
    test_feedback_sign();
    test_heading_error_sign();
    test_curvature_feedforward_sign();
    test_reset_clears_integrator();
    test_integrator_clamped();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
