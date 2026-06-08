/*
 * tests/test_tv.c
 *
 * Unit tests for torque_vectoring_update().
 *
 * Build and run via:   make test   (from the repo root)
 *
 * Each test_ function calls ASSERT() on a condition.  Any failure prints the
 * file/line and exits with a non-zero code so the build system can detect it.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../shared/tv_interface.h"
#include "../ECU_Firmware/include/torque_vectoring.h"

/* ---- Minimal test framework ---- */

static int g_tests  = 0;
static int g_passed = 0;

#define ASSERT(cond) do { \
    g_tests++; \
    if (cond) { \
        g_passed++; \
    } else { \
        fprintf(stderr, "FAIL  %s:%d  (%s)\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define ASSERT_NEAR(a, b, tol) ASSERT(fabsf((float)(a) - (float)(b)) <= (float)(tol))


/* ---- Helper: build a straight-ahead SensorData at given speed ---- */

static SensorData straight(float speed)
{
    SensorData s = {0};
    s.velocity       = speed;
    s.steering_angle = 0.0f;
    s.yaw_rate       = 0.0f;
    /* wheel speeds: all equal for straight line (no differential) */
    float w = (speed > 0.0f) ? speed / 0.254f : 0.0f; /* v / wheel_radius */
    s.wheel_speed[WHEEL_FL] = w;
    s.wheel_speed[WHEEL_FR] = w;
    s.wheel_speed[WHEEL_RL] = w;
    s.wheel_speed[WHEEL_RR] = w;
    return s;
}


/* ---- Tests ---- */

/*
 * Zero yaw error on a straight at speed -> perfectly even split.
 * desired = actual = 0, so bias = 0.
 */
static void test_zero_error_even_split(void)
{
    SensorData s = straight(10.0f);
    WheelTorques t;
    float total = 40.0f;   /* any positive value */
    torque_vectoring_update(&s, total, KP_YAW_DEFAULT, &t);

    float expected = total * 0.25f;
    ASSERT_NEAR(t.fl, expected, 0.01f);
    ASSERT_NEAR(t.fr, expected, 0.01f);
    ASSERT_NEAR(t.rl, expected, 0.01f);
    ASSERT_NEAR(t.rr, expected, 0.01f);
}

/*
 * When speed < 0.5 m/s, desired_yaw_rate is set to zero regardless of steer,
 * so there is no yaw error and the split is even.
 */
static void test_low_speed_no_yaw_demand(void)
{
    SensorData s = {0};
    s.velocity       = 0.1f;
    s.steering_angle = 0.5f;  /* large steer - should produce no demand */
    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, KP_YAW_DEFAULT, &t);

    float expected = 40.0f * 0.25f;
    ASSERT_NEAR(t.fl, expected, 0.01f);
    ASSERT_NEAR(t.fr, expected, 0.01f);
}

/*
 * Left turn (positive steering), positive yaw error -> right wheels get more
 * torque than left wheels (outer side).
 */
static void test_left_turn_biases_right_wheels(void)
{
    SensorData s = straight(10.0f);
    s.steering_angle = 0.3f;   /* turning left */
    s.yaw_rate       = 0.0f;   /* not rotating yet -> large positive error */
    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, KP_YAW_DEFAULT, &t);

    ASSERT(t.fr > t.fl);   /* right (outer) > left (inner) */
    ASSERT(t.rr > t.rl);
    /* The differential is split rear-biased (TV_REAR_SHARE), so the rear axle's
     * differential is larger than the front's - front == rear no longer holds. */
    ASSERT((t.rr - t.rl) > (t.fr - t.fl));
}

/*
 * Right turn (negative steering) -> left wheels get more torque.
 */
static void test_right_turn_biases_left_wheels(void)
{
    SensorData s = straight(10.0f);
    s.steering_angle = -0.3f;
    s.yaw_rate       =  0.0f;
    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, KP_YAW_DEFAULT, &t);

    ASSERT(t.fl > t.fr);
    ASSERT(t.rl > t.rr);
}

/*
 * The output must be symmetric: mirror the steer sign -> mirror the bias.
 */
static void test_symmetry(void)
{
    SensorData sL = straight(12.0f);
    SensorData sR = straight(12.0f);
    sL.steering_angle =  0.25f;
    sR.steering_angle = -0.25f;

    WheelTorques tL, tR;
    torque_vectoring_reset();
    torque_vectoring_update(&sL, 60.0f, KP_YAW_DEFAULT, &tL);
    torque_vectoring_reset();
    torque_vectoring_update(&sR, 60.0f, KP_YAW_DEFAULT, &tR);

    ASSERT_NEAR(tL.fl, tR.fr, 0.01f);
    ASSERT_NEAR(tL.fr, tR.fl, 0.01f);
    ASSERT_NEAR(tL.rl, tR.rr, 0.01f);
    ASSERT_NEAR(tL.rr, tR.rl, 0.01f);
}

/*
 * Deadband: a yaw error smaller than TV_YAW_DEADBAND produces an even split.
 */
static void test_deadband(void)
{
    SensorData s = {0};
    s.velocity       = 10.0f;
    s.steering_angle = 0.0f;
    /* Inject a tiny yaw rate error just inside the deadband */
    float desired = 10.0f * tanf(0.0f) / 1.55f;   /* = 0 for steer = 0 */
    s.yaw_rate = desired + (TV_YAW_DEADBAND * 0.5f); /* error = -half_band */
    for (int i = 0; i < 4; i++) s.wheel_speed[i] = 10.0f / 0.254f;

    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, KP_YAW_DEFAULT, &t);

    float base = 40.0f * 0.25f;
    ASSERT_NEAR(t.fl, base, 0.01f);
    ASSERT_NEAR(t.fr, base, 0.01f);
}

/*
 * Output clamp: even with a huge error the torques must not exceed the motor limits.
 */
static void test_clamp_upper(void)
{
    SensorData s = straight(5.0f);
    s.steering_angle = 0.6f;
    s.yaw_rate       = -5.0f;   /* extreme understeer -> huge positive error */
    WheelTorques t;
    torque_vectoring_update(&s, 200.0f, 500.0f, &t);   /* absurd gain */

    ASSERT(t.fl <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.fr <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.rl <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.rr <= MAX_MOTOR_TORQUE_NM + 0.001f);
}

static void test_clamp_lower(void)
{
    SensorData s = straight(5.0f);
    s.steering_angle = -0.6f;
    s.yaw_rate       =  5.0f;   /* extreme oversteer -> huge negative error */
    WheelTorques t;
    torque_vectoring_update(&s, 200.0f, 500.0f, &t);

    ASSERT(t.fl >= MIN_MOTOR_TORQUE_NM - 0.001f);
    ASSERT(t.fr >= MIN_MOTOR_TORQUE_NM - 0.001f);
    ASSERT(t.rl >= MIN_MOTOR_TORQUE_NM - 0.001f);
    ASSERT(t.rr >= MIN_MOTOR_TORQUE_NM - 0.001f);
}

/*
 * Gain = 0 must produce an even split regardless of yaw error.
 */
static void test_zero_gain(void)
{
    SensorData s = straight(15.0f);
    s.steering_angle = 0.4f;
    s.yaw_rate       = 0.0f;
    WheelTorques t;
    torque_vectoring_update(&s, 60.0f, 0.0f, &t);

    float base = 60.0f * 0.25f;
    ASSERT_NEAR(t.fl, base, 0.01f);
    ASSERT_NEAR(t.fr, base, 0.01f);
    ASSERT_NEAR(t.rl, base, 0.01f);
    ASSERT_NEAR(t.rr, base, 0.01f);
}

/*
 * Speed-dependent gain: at high speed the effective Kp is lower, so the bias
 * is smaller than at the reference speed with the same yaw error.
 */
static void test_speed_gain_scaling(void)
{
    /* No steering so desired_yaw_rate=0; error comes purely from yaw_rate offset.
     * Equal wheel speeds mean r_wheels=0, so yaw_rate_est = 0.75 * yaw_rate.
     * Setting yaw_rate = -0.5 gives yaw_error = 0 - 0.75*(-0.5) = +0.375 at
     * both speeds, isolating the speed-dependent gain effect. */
    SensorData sLow  = straight(6.0f);
    SensorData sHigh = straight(24.0f);
    sLow.yaw_rate  = -0.5f;
    sHigh.yaw_rate = -0.5f;

    WheelTorques tLow, tHigh;
    torque_vectoring_reset();
    torque_vectoring_update(&sLow,  40.0f, KP_YAW_DEFAULT, &tLow);
    torque_vectoring_reset();
    torque_vectoring_update(&sHigh, 40.0f, KP_YAW_DEFAULT, &tHigh);

    /* At low speed effective_kp = Kp*(12/6)=2*Kp; at high = Kp*(12/24)=0.5*Kp */
    float bias_low  = tLow.fr  - tLow.fl;
    float bias_high = tHigh.fr - tHigh.fl;
    ASSERT(bias_low > bias_high);
}


/*
 * Saturation-preserving clamp: with an extreme bias the outer wheel saturates
 * at the motor peak, but the differential (outer - inner) must be PRESERVED by
 * pushing the clipped amount onto the inner wheel (into regen) - not silently
 * collapsed.  High base torque so the outer side clips.
 */
static void test_saturation_preserves_differential(void)
{
    SensorData s = straight(8.0f);
    s.steering_angle = 0.5f;
    s.yaw_rate       = -3.0f;        /* large positive (understeer) error */
    WheelTorques t;
    torque_vectoring_update(&s, 104.0f, 300.0f, &t);  /* base ~26 Nm/wheel, saturating bias */

    /* Commanded rear differential = saturated bias (= max_bias) distributed by
     * the rear share, which is weighted by 2x (see torque_vectoring.c step 7):
     *   rear_diff = max_bias * (2 * TV_REAR_SHARE). */
    float max_bias  = MAX_MOTOR_TORQUE_NM * 0.5f;
    float want_diff = max_bias * (2.0f * TV_REAR_SHARE);
    ASSERT(t.rr <= MAX_MOTOR_TORQUE_NM + 0.001f);   /* outer clipped at peak */
    ASSERT(t.rl >= MIN_MOTOR_TORQUE_NM - 0.001f);   /* inner in regen */
    /* Differential preserved despite the clip (would collapse under a naive clamp). */
    ASSERT_NEAR(t.rr - t.rl, want_diff, 0.5f);
}


/* ---- Entry point ---- */

/* Run one test with a clean controller state.  The TV controller keeps internal
 * PID state across calls (correct for the continuous HIL loop, but it would let
 * one test case's residual integrator/derivative memory leak into the next), so
 * we reset between cases for isolation. */
#define RUN(fn) do { torque_vectoring_reset(); fn(); } while (0)

int main(void)
{
    RUN(test_zero_error_even_split);
    RUN(test_low_speed_no_yaw_demand);
    RUN(test_left_turn_biases_right_wheels);
    RUN(test_right_turn_biases_left_wheels);
    RUN(test_symmetry);
    RUN(test_deadband);
    RUN(test_clamp_upper);
    RUN(test_clamp_lower);
    RUN(test_zero_gain);
    RUN(test_speed_gain_scaling);
    RUN(test_saturation_preserves_differential);

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
