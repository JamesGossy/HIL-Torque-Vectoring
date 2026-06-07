/*
 * tests/test_vehicle_model.c
 *
 * Unit tests for vehicle_model_init() and vehicle_model_update().
 * Build and run via:  make test  (from the repo root)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../HIL_Firmware/include/vehicle_model.h"
#include "../shared/tv_interface.h"

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

/* ---- Helpers ---- */

static WheelTorques zero_torques(void)
{
    WheelTorques t = {0};
    return t;
}

static WheelTorques equal_torques(float nm)
{
    WheelTorques t;
    t.fl = t.fr = t.rl = t.rr = nm;
    return t;
}

/* ---- Tests ---- */

/* init zeroes all motion and places the car at the given pose */
static void test_init_zeros_state(void)
{
    VehicleState s;
    vehicle_model_init(&s, 10.0f, 20.0f, 1.0f);

    ASSERT_NEAR(s.x,        10.0f, 1e-5f);
    ASSERT_NEAR(s.y,        20.0f, 1e-5f);
    ASSERT_NEAR(s.heading,   1.0f, 1e-5f);
    ASSERT_NEAR(s.velocity,  0.0f, 1e-5f);
    ASSERT_NEAR(s.vy,        0.0f, 1e-5f);
    ASSERT_NEAR(s.yaw_rate,  0.0f, 1e-5f);
    ASSERT_NEAR(s.ax,        0.0f, 1e-5f);
    ASSERT_NEAR(s.ay,        0.0f, 1e-5f);
    ASSERT_NEAR(s.ax_filt,   0.0f, 1e-5f);
    ASSERT_NEAR(s.ay_filt,   0.0f, 1e-5f);
    for (int i = 0; i < 4; i++)
        ASSERT_NEAR(s.wheelspeed[i], 0.0f, 1e-5f);
}

/* a stationary car with zero torque must not move */
static void test_zero_torque_no_motion(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    WheelTorques t = zero_torques();

    for (int i = 0; i < 100; i++)
        vehicle_model_update(&s, &t, 0.01f);

    ASSERT_NEAR(s.x,       0.0f, 0.01f);
    ASSERT_NEAR(s.y,       0.0f, 0.01f);
    ASSERT_NEAR(s.velocity, 0.0f, 0.01f);
}

/* positive equal torques on all wheels must accelerate the car forward */
static void test_equal_torque_accelerates(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    WheelTorques t = equal_torques(20.0f);

    float v_before = s.velocity;
    for (int i = 0; i < 50; i++)
        vehicle_model_update(&s, &t, 0.01f);

    ASSERT(s.velocity > v_before);
    ASSERT(s.x > 0.0f);
    ASSERT_NEAR(s.y, 0.0f, 0.05f);   /* straight-ahead, no lateral drift */
}

/* speed is clamped to MAX_SPEED_MS regardless of torque magnitude */
static void test_speed_clamp(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    WheelTorques t = equal_torques(200.0f);   /* unrealistically large */

    for (int i = 0; i < 1000; i++)
        vehicle_model_update(&s, &t, 0.01f);

    ASSERT(s.velocity <= MAX_SPEED_MS + 0.001f);
}

/* straight-ahead travel: heading stays near zero, y stays near zero */
static void test_straight_line_heading(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    WheelTorques t = equal_torques(15.0f);

    for (int i = 0; i < 200; i++)
        vehicle_model_update(&s, &t, 0.01f);

    ASSERT_NEAR(s.heading, 0.0f, 0.05f);
    ASSERT_NEAR(s.y,       0.0f, 0.1f);
}

/* left-steering generates positive yaw rate and leftward lateral motion */
static void test_left_steer_yaws_left(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    s.velocity = 10.0f;   /* pre-seed speed so the tyre forces are active */
    s.steering = 0.2f;
    WheelTorques t = equal_torques(10.0f);

    for (int i = 0; i < 50; i++)
        vehicle_model_update(&s, &t, 0.01f);

    ASSERT(s.yaw_rate > 0.0f);
    ASSERT(s.heading  > 0.0f);
}

/* symmetry: mirroring steer sign mirrors the resulting yaw rate sign */
static void test_steer_symmetry(void)
{
    VehicleState sL, sR;
    vehicle_model_init(&sL, 0.0f, 0.0f, 0.0f);
    vehicle_model_init(&sR, 0.0f, 0.0f, 0.0f);
    sL.velocity = sR.velocity = 10.0f;
    sL.steering =  0.2f;
    sR.steering = -0.2f;
    WheelTorques t = equal_torques(10.0f);

    for (int i = 0; i < 50; i++) {
        vehicle_model_update(&sL, &t, 0.01f);
        vehicle_model_update(&sR, &t, 0.01f);
    }

    ASSERT_NEAR(sL.yaw_rate, -sR.yaw_rate, 0.05f);
}

/* negative torque on a moving car must decelerate it */
static void test_braking_decelerates(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    s.velocity = 15.0f;
    WheelTorques t = equal_torques(-20.0f);

    float v_before = s.velocity;
    for (int i = 0; i < 50; i++)
        vehicle_model_update(&s, &t, 0.01f);

    ASSERT(s.velocity < v_before);
    ASSERT(s.velocity >= 0.0f);   /* must not go negative */
}

/* wheel speeds must be positive and proportional to forward speed */
static void test_wheelspeed_positive_at_speed(void)
{
    VehicleState s;
    vehicle_model_init(&s, 0.0f, 0.0f, 0.0f);
    s.velocity = 10.0f;
    WheelTorques t = equal_torques(10.0f);
    vehicle_model_update(&s, &t, 0.01f);

    for (int i = 0; i < 4; i++)
        ASSERT(s.wheelspeed[i] > 0.0f);
}

/* ---- Entry point ---- */

int main(void)
{
    test_init_zeros_state();
    test_zero_torque_no_motion();
    test_equal_torque_accelerates();
    test_speed_clamp();
    test_straight_line_heading();
    test_left_steer_yaws_left();
    test_steer_symmetry();
    test_braking_decelerates();
    test_wheelspeed_positive_at_speed();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
