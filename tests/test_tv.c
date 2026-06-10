/*
 * Unit tests for torque_vectoring_update(). Build and run with: make test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../shared/tv_interface.h"
#include "../ECU_Firmware/include/torque_vectoring.h"

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


// Build a straight-ahead SensorData at the given speed.
static SensorData straight(float speed)
{
    SensorData s     = { 0 };
    s.velocity       = speed;
    s.steering_angle = 0.0f;
    s.yaw_rate       = 0.0f;
    float w                 = (speed > 0.0f) ? speed / 0.254f : 0.0f; // v / wheel_radius
    s.wheel_speed[WHEEL_FL] = w;
    s.wheel_speed[WHEEL_FR] = w;
    s.wheel_speed[WHEEL_RL] = w;
    s.wheel_speed[WHEEL_RR] = w;
    return s;
}


// Zero yaw error on a straight gives equal left/right totals and delivers the full demand.
static void test_zero_error_even_split(void)
{
    SensorData s = straight(10.0f);
    WheelTorques t;
    float total = 40.0f;
    torque_vectoring_update(&s, total, g_KP_YAW, &t);

    ASSERT_NEAR(t.fl + t.rl, t.fr + t.rr, 0.01f);
    ASSERT_NEAR(t.fl + t.fr + t.rl + t.rr, total, 0.5f);
}

// Below 0.5 m/s the desired yaw rate is zero, so there is no differential.
static void test_low_speed_no_yaw_demand(void)
{
    SensorData s     = { 0 };
    s.velocity       = 0.1f;
    s.steering_angle = 0.5f;
    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, g_KP_YAW, &t);

    ASSERT_NEAR(t.fl + t.rl, t.fr + t.rr, 0.01f);
}

// A left turn biases the right (outer) wheels.
static void test_left_turn_biases_right_wheels(void)
{
    SensorData s     = straight(10.0f);
    s.steering_angle = 0.3f;
    s.yaw_rate       = 0.0f;
    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, g_KP_YAW, &t);

    ASSERT(t.fr > t.fl);
    ASSERT(t.rr > t.rl);
    ASSERT((t.fr + t.rr) > (t.fl + t.rl)); // outer side carries more total
}

// A right turn biases the left wheels.
static void test_right_turn_biases_left_wheels(void)
{
    SensorData s     = straight(10.0f);
    s.steering_angle = -0.3f;
    s.yaw_rate       = 0.0f;
    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, g_KP_YAW, &t);

    ASSERT(t.fl > t.fr);
    ASSERT(t.rl > t.rr);
}

// Output is symmetric: mirroring the steer sign mirrors the bias.
static void test_symmetry(void)
{
    SensorData sL     = straight(12.0f);
    SensorData sR     = straight(12.0f);
    sL.steering_angle = 0.25f;
    sR.steering_angle = -0.25f;

    WheelTorques tL, tR;
    torque_vectoring_reset();
    torque_vectoring_update(&sL, 60.0f, g_KP_YAW, &tL);
    torque_vectoring_reset();
    torque_vectoring_update(&sR, 60.0f, g_KP_YAW, &tR);

    ASSERT_NEAR(tL.fl, tR.fr, 0.01f);
    ASSERT_NEAR(tL.fr, tR.fl, 0.01f);
    ASSERT_NEAR(tL.rl, tR.rr, 0.01f);
    ASSERT_NEAR(tL.rr, tR.rl, 0.01f);
}

// A yaw error inside the deadband produces an even split.
static void test_deadband(void)
{
    SensorData s     = { 0 };
    s.velocity       = 10.0f;
    s.steering_angle = 0.0f;
    float desired = 10.0f * tanf(0.0f) / 1.55f;           // = 0 for steer = 0
    s.yaw_rate    = desired + (g_TV_YAW_DEADBAND * 0.5f);  // error inside the band
    for (int i = 0; i < 4; i++)
        s.wheel_speed[i] = 10.0f / 0.254f;

    WheelTorques t;
    torque_vectoring_update(&s, 40.0f, g_KP_YAW, &t);

    ASSERT_NEAR(t.fl + t.rl, t.fr + t.rr, 0.01f);
}

// A huge error must not push any wheel above the motor limit.
static void test_clamp_upper(void)
{
    SensorData s     = straight(5.0f);
    s.steering_angle = 0.6f;
    s.yaw_rate       = -5.0f; // extreme understeer error
    WheelTorques t;
    torque_vectoring_update(&s, 200.0f, 500.0f, &t); // absurd gain

    ASSERT(t.fl <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.fr <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.rl <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.rr <= MAX_MOTOR_TORQUE_NM + 0.001f);
}

// A huge negative error must not push any wheel below the motor limit.
static void test_clamp_lower(void)
{
    SensorData s     = straight(5.0f);
    s.steering_angle = -0.6f;
    s.yaw_rate       = 5.0f; // extreme oversteer error
    WheelTorques t;
    torque_vectoring_update(&s, 200.0f, 500.0f, &t);

    ASSERT(t.fl >= MIN_MOTOR_TORQUE_NM - 0.001f);
    ASSERT(t.fr >= MIN_MOTOR_TORQUE_NM - 0.001f);
    ASSERT(t.rl >= MIN_MOTOR_TORQUE_NM - 0.001f);
    ASSERT(t.rr >= MIN_MOTOR_TORQUE_NM - 0.001f);
}

// Gain 0 disables the yaw moment, so there is no differential and the full total is delivered.
static void test_zero_gain(void)
{
    SensorData s     = straight(15.0f);
    s.steering_angle = 0.4f;
    s.yaw_rate       = 0.0f;
    WheelTorques t;
    torque_vectoring_update(&s, 60.0f, 0.0f, &t);

    ASSERT_NEAR(t.fl + t.rl, t.fr + t.rr, 0.01f);
    ASSERT_NEAR(t.fl + t.fr + t.rl + t.rr, 60.0f, 0.5f);
}

// Higher speed lowers the effective gain, so the bias is smaller for the same yaw error.
static void test_speed_gain_scaling(void)
{
    // Tiny yaw error and small gain keep both speeds in the linear, unsaturated regime.
    SensorData sLow  = straight(6.0f);
    SensorData sHigh = straight(24.0f);
    sLow.yaw_rate    = -0.1f;
    sHigh.yaw_rate   = -0.1f;

    const float kp = 5.0f; // small gain keeps both cases below max_bias
    WheelTorques tLow, tHigh;
    torque_vectoring_reset();
    torque_vectoring_update(&sLow, 40.0f, kp, &tLow);
    torque_vectoring_reset();
    torque_vectoring_update(&sHigh, 40.0f, kp, &tHigh);

    float bias_low  = tLow.fr - tLow.fl;
    float bias_high = tHigh.fr - tHigh.fl;
    ASSERT(bias_low > bias_high);
}


// Under saturation the moment is held as a left/right differential equal to the clamped bias.
static void test_saturation_preserves_differential(void)
{
    SensorData s     = straight(8.0f);
    s.steering_angle = 0.5f;
    s.yaw_rate       = -3.0f; // large positive (understeer) error
    WheelTorques t;
    torque_vectoring_update(&s, 104.0f, 300.0f, &t); // large demand, saturating bias

    float left     = t.fl + t.rl;
    float right    = t.fr + t.rr;
    float max_bias = MAX_MOTOR_TORQUE_NM * 0.5f;
    ASSERT(right > left);
    ASSERT_NEAR(right - left, max_bias, 0.5f); // differential not collapsed by clipping
    ASSERT(t.fr <= MAX_MOTOR_TORQUE_NM + 0.001f);
    ASSERT(t.rr <= MAX_MOTOR_TORQUE_NM + 0.001f);
}


// Run one test after resetting the controller's internal PID state for isolation.
#define RUN(fn)                                                                                    \
    do {                                                                                           \
        torque_vectoring_reset();                                                                  \
        fn();                                                                                      \
    } while (0)

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
