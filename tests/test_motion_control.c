/* Unit tests for motion_control_update(). Build and run via: make test. */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "../HIL_Firmware/include/motion_control.h"
#include "../HIL_Firmware/include/track_parser.h"
#include "../HIL_Firmware/include/vehicle_model.h"

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

/* Build a straight east-pointing track with boundary cones at y = +/-half_w. */
static Track make_straight_track(float length, float half_w)
{
    Track t;
    memset(&t, 0, sizeof(t));
    int n = 20;
    for (int i = 0; i < n; i++) {
        float x            = length * i / (float)(n - 1);
        t.points[i].x      = x;
        t.points[i].y      = 0.0f;
        t.left_cones[i].x  = x;
        t.left_cones[i].y  = half_w;
        t.right_cones[i].x = x;
        t.right_cones[i].y = -half_w;
    }
    t.count         = n;
    t.left_count    = n;
    t.right_count   = n;
    t.current_index = 0;
    return t;
}

static VehicleState make_state(float x, float y, float heading, float speed)
{
    VehicleState s;
    vehicle_model_init(&s, x, y, heading);
    s.velocity = speed;
    return s;
}

/* An on-path car below target speed must get positive throttle. */
static void test_below_target_speed_gives_throttle(void)
{
    Track t        = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, 0.0f, 0.0f, 5.0f); // well below TARGET_SPEED_MS

    float torque = motion_control_update(&s, &t, NULL);
    ASSERT(torque > 0.0f);
}

/* A car above target speed must get a braking demand. */
static void test_above_target_speed_gives_brake(void)
{
    Track t        = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, 0.0f, 0.0f, TARGET_SPEED_MS + 5.0f);

    float torque = motion_control_update(&s, &t, NULL);
    ASSERT(torque < 0.0f);
}

/* Throttle must never exceed DRIVER_TORQUE_NM. */
static void test_throttle_clamped(void)
{
    Track t        = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, 0.0f, 0.0f, 0.0f); // stationary, max demand

    float torque = motion_control_update(&s, &t, NULL);
    ASSERT(torque <= DRIVER_TORQUE_NM + 0.001f);
}

/* Braking demand must never exceed DRIVER_BRAKE_NM in magnitude. */
static void test_brake_clamped(void)
{
    Track t        = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, 0.0f, 0.0f, TARGET_SPEED_MS * 3.0f);

    float torque = motion_control_update(&s, &t, NULL);
    ASSERT(torque >= DRIVER_BRAKE_NM - 0.001f);
}

/* Steering must stay within +/-g_MAX_STEER_RAD. */
static void test_steer_clamped(void)
{
    Track t = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, -4.0f, 0.0f, 10.0f); // offset right for large cross-track error

    for (int i = 0; i < 20; i++)
        motion_control_update(&s, &t, NULL);

    ASSERT(s.steering <= g_MAX_STEER_RAD + 0.001f);
    ASSERT(s.steering >= -g_MAX_STEER_RAD - 0.001f);
}

/* On-path car pointing straight should keep steering near zero. */
static void test_on_path_small_steer(void)
{
    Track t        = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(5.0f, 0.0f, 0.0f, 10.0f);

    motion_control_update(&s, &t, NULL);
    ASSERT(fabsf(s.steering) < 0.15f);
}

/* out_target_speed is written when a non-NULL pointer is passed. */
static void test_out_target_speed_written(void)
{
    Track t        = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, 0.0f, 0.0f, 10.0f);
    float target   = -999.0f;

    motion_control_update(&s, &t, &target);
    ASSERT(target >= 0.0f);
    ASSERT(target <= TARGET_SPEED_MS + 0.001f);
}

/* Commanded steering must not jump more than one tick's max step. */
static void test_steer_slew_rate(void)
{
    Track t = make_straight_track(50.0f, 2.5f);
    VehicleState s = make_state(0.0f, -3.0f, 0.0f, 10.0f); // large offset forces max steer demand
    float max_step = g_MAX_STEER_RATE_RADS * CONTROL_DT_S;

    float prev_steer = s.steering;
    for (int i = 0; i < 10; i++) {
        motion_control_update(&s, &t, NULL);
        ASSERT(fabsf(s.steering - prev_steer) <= max_step + 1e-4f);
        prev_steer = s.steering;
    }
}

int main(void)
{
    test_below_target_speed_gives_throttle();
    test_above_target_speed_gives_brake();
    test_throttle_clamped();
    test_brake_clamped();
    test_steer_clamped();
    test_on_path_small_steer();
    test_out_target_speed_written();
    test_steer_slew_rate();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
