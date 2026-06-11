/* Unit tests for motion_control_update(). Build and run via: make test. */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "../ECU_Firmware/include/motion_control.h"
#include "../ECU_Firmware/include/ecu_map.h"

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

/* Build a straight east-pointing map with boundary cones at y = +/-half_w. */
static EcuMap make_straight_map(float length, float half_w)
{
    EcuMap m;
    memset(&m, 0, sizeof(m));
    int n = 20;
    for (int i = 0; i < n; i++) {
        float x            = length * i / (float)(n - 1);
        m.points[i].x      = x;
        m.points[i].y      = 0.0f;
        m.left_cones[i].x  = x;
        m.left_cones[i].y  = half_w;
        m.right_cones[i].x = x;
        m.right_cones[i].y = -half_w;
    }
    m.count         = n;
    m.left_count    = n;
    m.right_count   = n;
    m.current_index = 0;
    return m;
}

static CtrlPose make_pose(float x, float y, float heading, float speed)
{
    CtrlPose p = { x, y, heading, speed, 0.0f, 0.0f, 0.0f };
    return p;
}

/* An on-path car below target speed must get positive throttle. */
static void test_below_target_speed_gives_throttle(void)
{
    EcuMap m      = make_straight_map(50.0f, 2.5f);
    CtrlPose p    = make_pose(0.0f, 0.0f, 0.0f, 5.0f); // well below TARGET_SPEED_MS
    float steer   = 0.0f;
    float torque  = motion_control_update(&p, &m, &steer, NULL);
    ASSERT(torque > 0.0f);
}

/* A car above target speed must get a braking demand. */
static void test_above_target_speed_gives_brake(void)
{
    EcuMap m     = make_straight_map(50.0f, 2.5f);
    CtrlPose p   = make_pose(0.0f, 0.0f, 0.0f, TARGET_SPEED_MS + 5.0f);
    float steer  = 0.0f;
    float torque = motion_control_update(&p, &m, &steer, NULL);
    ASSERT(torque < 0.0f);
}

/* Throttle must never exceed DRIVER_TORQUE_NM. */
static void test_throttle_clamped(void)
{
    EcuMap m     = make_straight_map(50.0f, 2.5f);
    CtrlPose p   = make_pose(0.0f, 0.0f, 0.0f, 0.0f); // stationary, max demand
    float steer  = 0.0f;
    float torque = motion_control_update(&p, &m, &steer, NULL);
    ASSERT(torque <= DRIVER_TORQUE_NM + 0.001f);
}

/* Braking demand must never exceed DRIVER_BRAKE_NM in magnitude. */
static void test_brake_clamped(void)
{
    EcuMap m     = make_straight_map(50.0f, 2.5f);
    CtrlPose p   = make_pose(0.0f, 0.0f, 0.0f, TARGET_SPEED_MS * 3.0f);
    float steer  = 0.0f;
    float torque = motion_control_update(&p, &m, &steer, NULL);
    ASSERT(torque >= DRIVER_BRAKE_NM - 0.001f);
}

/* Steering must stay within +/-g_MAX_STEER_RAD. */
static void test_steer_clamped(void)
{
    EcuMap m   = make_straight_map(50.0f, 2.5f);
    CtrlPose p = make_pose(0.0f, -4.0f, 0.0f, 10.0f); // offset right for large cross-track error
    float steer = 0.0f;

    for (int i = 0; i < 20; i++) {
        motion_control_update(&p, &m, &steer, NULL);
        p.steering = steer; // carry the applied angle forward for the slew limit
    }

    ASSERT(steer <= g_MAX_STEER_RAD + 0.001f);
    ASSERT(steer >= -g_MAX_STEER_RAD - 0.001f);
}

/* On-path car pointing straight should keep steering near zero. */
static void test_on_path_small_steer(void)
{
    EcuMap m    = make_straight_map(50.0f, 2.5f);
    CtrlPose p  = make_pose(5.0f, 0.0f, 0.0f, 10.0f);
    float steer = 0.0f;

    motion_control_update(&p, &m, &steer, NULL);
    ASSERT(fabsf(steer) < 0.15f);
}

/* out_target_speed is written when a non-NULL pointer is passed. */
static void test_out_target_speed_written(void)
{
    EcuMap m     = make_straight_map(50.0f, 2.5f);
    CtrlPose p   = make_pose(0.0f, 0.0f, 0.0f, 10.0f);
    float steer  = 0.0f;
    float target = -999.0f;

    motion_control_update(&p, &m, &steer, &target);
    ASSERT(target >= 0.0f);
    ASSERT(target <= TARGET_SPEED_MS + 0.001f);
}

/* Commanded steering must not jump more than one tick's max step. */
static void test_steer_slew_rate(void)
{
    EcuMap m   = make_straight_map(50.0f, 2.5f);
    CtrlPose p = make_pose(0.0f, -3.0f, 0.0f, 10.0f); // large offset forces max steer demand
    float max_step = g_MAX_STEER_RATE_RADS * CONTROL_DT_S;

    float steer      = 0.0f;
    float prev_steer = 0.0f;
    for (int i = 0; i < 10; i++) {
        motion_control_update(&p, &m, &steer, NULL);
        ASSERT(fabsf(steer - prev_steer) <= max_step + 1e-4f);
        prev_steer = steer;
        p.steering = steer;
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
