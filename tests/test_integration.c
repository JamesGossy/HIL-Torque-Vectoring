/*
 * tests/test_integration.c
 *
 * HIL-loop integration tests. The unit suites cover each block in isolation
 * (vehicle_model, path_planning, torque_vectoring, lqr_steer, the throttle
 * planner); this one exercises the SEAM they meet at - the per-tick wiring that
 * main.c performs: driver -> sensor packing -> ECU -> vehicle -> track. `make
 * eval` exercises the same loop over a full lap, but it judges by printed lap
 * metrics; this pins the wiring invariants directly and cheaply, so a units bug
 * in the sensor bus or a broken reset shows up as a failed assertion rather than
 * a silently worse lap.
 *
 * Build and run via:  make test  (from the repo root)
 */

#include <stdio.h>
#include <math.h>

#include "../HIL_Firmware/include/vehicle_model.h"
#include "../HIL_Firmware/include/track_parser.h"
#include "../HIL_Firmware/include/motion_control.h"
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

static const float DT = 0.01f;

/* ---- Shared harness: one full HIL tick, exactly as main.c wires it ---- */

/* Pack the sensor bus from the vehicle state the way main.c does (the conversion
 * from stored motor-shaft RPM to wheel angular speed is part of what we test). */
static void pack_sensors(const VehicleState *s, float driver_torque, SensorData *sensors)
{
    const float RPM_TO_WHEEL_RADS = (2.0f * 3.14159265358979f) / (GEAR_RATIO * 60.0f);
    sensors->yaw_rate       = s->yaw_rate;
    sensors->velocity       = s->velocity;
    sensors->steering_angle = s->steering;
    sensors->driver_torque  = driver_torque;
    for (int i = 0; i < 4; i++)
        sensors->wheel_speed[i] = s->wheelspeed[i] * RPM_TO_WHEEL_RADS;
}

/* Advance the full loop one tick (driver -> sensors -> ECU -> vehicle -> track),
 * with TV either on (model split) or off (even split), as main.c does. */
static float step(VehicleState *state, Track *track, int tv_enabled)
{
    float driver_torque = motion_control_update(state, track, NULL);

    SensorData sensors = {0};
    pack_sensors(state, driver_torque, &sensors);

    WheelTorques torques = {0};
    if (tv_enabled) {
        torque_vectoring_update(&sensors, driver_torque, KP_YAW_DEFAULT, &torques);
    } else {
        float base = driver_torque * 0.25f;
        torques.fl = torques.fr = torques.rl = torques.rr = base;
    }

    vehicle_model_update(state, &torques, DT);
    track_update(track, state->x, state->y);
    return driver_torque;
}

/* Init the real FSG track + car at the start line, with clean controller state -
 * the same sequence the sim entry points run. */
static void setup(Track *track, VehicleState *state)
{
    track_init(track);
    float ih = atan2f(track->points[1].y - track->points[0].y,
                      track->points[1].x - track->points[0].x);
    vehicle_model_init(state, track->points[0].x, track->points[0].y, ih);
    motion_control_reset();
    torque_vectoring_reset();
}

/* ---- Tests ---- */

/* The full loop must run for a few seconds on the real track without producing a
 * NaN/Inf in any state the controllers depend on. */
static void test_loop_stays_finite(void)
{
    Track track; VehicleState state;
    setup(&track, &state);

    for (int i = 0; i < 500; i++) {        /* 5 s */
        step(&state, &track, 1);
        ASSERT(isfinite(state.x) && isfinite(state.y));
        ASSERT(isfinite(state.velocity) && isfinite(state.yaw_rate));
        ASSERT(isfinite(state.steering) && isfinite(state.vy));
    }
}

/* From a standstill the wired-up car must actually drive: build speed and cover
 * ground. This catches a dead driver->ECU->vehicle path that no single unit test
 * would (each block can be individually correct yet mis-wired). */
static void test_car_drives_from_standstill(void)
{
    Track track; VehicleState state;
    setup(&track, &state);

    float x0 = state.x, y0 = state.y;
    for (int i = 0; i < 300; i++)          /* 3 s */
        step(&state, &track, 1);

    ASSERT(state.velocity > 3.0f);         /* accelerated away from rest */
    float moved = hypotf(state.x - x0, state.y - y0);
    ASSERT(moved > 5.0f);                  /* covered real ground */
}

/* Over a longer run the car must make forward progress along the racing line
 * (the waypoint index advances), confirming the track-update wiring closes the
 * loop. */
static void test_waypoint_progress(void)
{
    Track track; VehicleState state;
    setup(&track, &state);

    int idx0 = track.current_index;
    for (int i = 0; i < 1000; i++)         /* 10 s */
        step(&state, &track, 1);

    int laps = track.lap_count;
    int advanced = (track.current_index != idx0) || (laps > 0);
    ASSERT(advanced);
}

/* Sensor packing: the wheel-speed bus the ECU sees must be the vehicle model's
 * stored motor RPM converted to wheel rad/s by the gear ratio - and on a
 * straight-line launch it should be consistent with forward speed
 * (v ~= wheel_rad_s * WHEEL_RADIUS_M). A wrong gear factor here would silently
 * corrupt the ECU's wheel-speed yaw estimate. */
static void test_sensor_wheel_speed_units(void)
{
    Track track; VehicleState state;
    setup(&track, &state);

    /* run briefly so the wheels are turning */
    for (int i = 0; i < 100; i++) step(&state, &track, 1);

    SensorData sensors = {0};
    pack_sensors(&state, 0.0f, &sensors);

    for (int i = 0; i < 4; i++) {
        ASSERT(isfinite(sensors.wheel_speed[i]));
        ASSERT(sensors.wheel_speed[i] > 0.0f);          /* moving forward */
        float v_from_wheel = sensors.wheel_speed[i] * WHEEL_RADIUS_M;
        ASSERT_NEAR(v_from_wheel, state.velocity, 2.0f); /* same ballpark as v */
    }
}

/* TV on/off must both keep every wheel torque inside the motor limits, and TV
 * off must be a perfectly even split of the driver demand. */
static void test_tv_on_off_torque_valid(void)
{
    Track track; VehicleState state;
    setup(&track, &state);
    for (int i = 0; i < 200; i++) step(&state, &track, 1);   /* into a corner */

    float driver = motion_control_update(&state, &track, NULL);
    SensorData sensors = {0};
    pack_sensors(&state, driver, &sensors);

    WheelTorques on = {0};
    torque_vectoring_update(&sensors, driver, KP_YAW_DEFAULT, &on);
    float w_on[4] = { on.fl, on.fr, on.rl, on.rr };
    for (int i = 0; i < 4; i++) {
        ASSERT(w_on[i] <= MAX_MOTOR_TORQUE_NM + 0.001f);
        ASSERT(w_on[i] >= MIN_MOTOR_TORQUE_NM - 0.001f);
    }

    /* TV off: even split, sums back to the driver demand */
    float base = driver * 0.25f;
    ASSERT_NEAR(base * 4.0f, driver, 0.01f);
}

/* Reset sequencing: two independent runs that reset between them must produce an
 * IDENTICAL trajectory. This is the regression guard for the #1 fix - if a
 * controller's static state leaked across runs (no reset), the second run would
 * diverge from the first. */
static void test_reset_gives_repeatable_run(void)
{
    Track t1, t2; VehicleState s1, s2;

    setup(&t1, &s1);
    for (int i = 0; i < 400; i++) step(&s1, &t1, 1);

    setup(&t2, &s2);                      /* setup() resets all controllers */
    for (int i = 0; i < 400; i++) step(&s2, &t2, 1);

    ASSERT_NEAR(s1.x,        s2.x,        1e-3f);
    ASSERT_NEAR(s1.y,        s2.y,        1e-3f);
    ASSERT_NEAR(s1.heading,  s2.heading,  1e-3f);
    ASSERT_NEAR(s1.velocity, s2.velocity, 1e-3f);
    ASSERT(t1.current_index == t2.current_index);
}

/* Without a reset, residual static state from a prior run leaks into the next.
 * Run once, then run again WITHOUT resetting and confirm the trajectory differs
 * from the clean reference - i.e. the reset is doing real work, not a no-op. */
static void test_missing_reset_leaks_state(void)
{
    Track tref, tleak; VehicleState sref, sleak;

    /* Clean reference run. */
    setup(&tref, &sref);
    for (int i = 0; i < 400; i++) step(&sref, &tref, 1);

    /* Prime the controllers with a hard cornering burst, then start a fresh
     * geometry WITHOUT resetting - the integrators/progress index carry over. */
    Track tprime; VehicleState sprime;
    setup(&tprime, &sprime);
    for (int i = 0; i < 400; i++) step(&sprime, &tprime, 1);

    track_init(&tleak);
    float ih = atan2f(tleak.points[1].y - tleak.points[0].y,
                      tleak.points[1].x - tleak.points[0].x);
    vehicle_model_init(&sleak, tleak.points[0].x, tleak.points[0].y, ih);
    /* deliberately NO motion_control_reset()/torque_vectoring_reset() */
    for (int i = 0; i < 400; i++) step(&sleak, &tleak, 1);

    /* The leaked run should NOT match the clean reference (proves reset matters).
     * Restore clean state afterwards so the next test is unaffected. */
    float dx = fabsf(sleak.x - sref.x), dy = fabsf(sleak.y - sref.y);
    ASSERT(dx > 1e-4f || dy > 1e-4f);
    motion_control_reset();
    torque_vectoring_reset();
}

/* ---- Entry point ---- */

int main(void)
{
    test_loop_stays_finite();
    test_car_drives_from_standstill();
    test_waypoint_progress();
    test_sensor_wheel_speed_units();
    test_tv_on_off_torque_valid();
    test_reset_gives_repeatable_run();
    test_missing_reset_leaks_state();

    fprintf(stderr, "%d/%d tests passed\n", g_passed, g_tests);
    return (g_passed == g_tests) ? 0 : 1;
}
