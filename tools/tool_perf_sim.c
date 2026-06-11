/*
 * Compute-speed benchmark. Runs the tick loop with no real-time sleep for a
 * fixed wall-clock time and reports throughput. Measures host speed, not
 * driving quality. Build with `make perf`.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../HIL_Firmware/include/vehicle_model.h"
#include "../HIL_Firmware/include/track_parser.h"
#include "../shared/tv_interface.h"
#include "../ECU_Firmware/include/torque_vectoring.h"
#include "../ECU_Firmware/include/motion_control.h"
#include "../ECU_Firmware/include/ecu_map.h"

#define DT 0.01f /* simulated seconds per tick, 100 Hz */

/* Copy a ground-truth Track into the ECU map type for the legacy driver. */
static void track_to_ecu_map(const Track *t, EcuMap *m)
{
    int i;
    m->count         = t->count;
    m->current_index = t->current_index;
    m->lap_count     = t->lap_count;
    for (i = 0; i < t->count; i++) {
        m->points[i].x = t->points[i].x;
        m->points[i].y = t->points[i].y;
    }
    m->left_count = t->left_count;
    for (i = 0; i < t->left_count; i++) {
        m->left_cones[i].x = t->left_cones[i].x;
        m->left_cones[i].y = t->left_cones[i].y;
    }
    m->right_count = t->right_count;
    for (i = 0; i < t->right_count; i++) {
        m->right_cones[i].x = t->right_cones[i].x;
        m->right_cones[i].y = t->right_cones[i].y;
    }
}

// Monotonic wall-clock time in seconds.
#ifdef _WIN32
#include <windows.h>
static double wall_s(void)
{
    static LARGE_INTEGER freq = { 0 };
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)freq.QuadPart;
}
#else
#include <time.h>
static double wall_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
#endif

// Runs the tick loop for the budget and prints throughput numbers.
int main(int argc, char **argv)
{
    double budget_s = 1.0; // override with argv[1]
    if (argc > 1) {
        double v = atof(argv[1]);
        if (v > 0.0) budget_s = v;
    }

    Track track;
    EcuMap ecu_map;
    VehicleState state;
    WheelTorques torques = { 0 };
    SensorData sensors   = { 0 };

    track_init(&track);
    track_to_ecu_map(&track, &ecu_map);

    float ih = atan2f(track.points[1].y - track.points[0].y, track.points[1].x - track.points[0].x);
    vehicle_model_init(&state, track.points[0].x, track.points[0].y, ih);

    motion_control_reset(); // clear driver and ECU state for this run
    torque_vectoring_reset();

    const float R2W = (2.0f * 3.14159265358979f) / (GEAR_RATIO * 60.0f);
    int start_lap   = track.lap_count;
    long ticks      = 0;

    const long CHECK_EVERY = 256; // batch ticks so the clock call does not dominate

    double t0  = wall_s();
    double now = t0;
    for (;;) {
        for (long b = 0; b < CHECK_EVERY; b++) {
            float target_speed = 0.0f;
            float steer_cmd    = state.steering;
            CtrlPose pose      = { state.x, state.y, state.heading, state.velocity, state.yaw_rate,
                state.ay_filt, state.steering };
            float dq           = motion_control_update(&pose, &ecu_map, &steer_cmd, &target_speed);
            state.steering     = steer_cmd;

            sensors.yaw_rate              = state.yaw_rate;
            sensors.velocity              = state.velocity;
            sensors.steering_angle        = state.steering;
            sensors.driver_torque         = dq;
            sensors.wheel_speed[WHEEL_FL] = state.wheelspeed[WHEEL_FL] * R2W;
            sensors.wheel_speed[WHEEL_FR] = state.wheelspeed[WHEEL_FR] * R2W;
            sensors.wheel_speed[WHEEL_RL] = state.wheelspeed[WHEEL_RL] * R2W;
            sensors.wheel_speed[WHEEL_RR] = state.wheelspeed[WHEEL_RR] * R2W;

            torque_vectoring_update(&sensors, dq, g_KP_YAW, &torques);
            vehicle_model_update(&state, &torques, DT);
            track_update(&track, state.x, state.y);
            ticks++;
        }
        now = wall_s();
        if (now - t0 >= budget_s) break;
    }

    double wall      = now - t0;
    double sim_s     = ticks * (double)DT;
    int laps         = track.lap_count - start_lap;
    double rt_factor = sim_s / wall;
    double ticks_ps  = ticks / wall;
    double laps_ps   = laps / wall;

    printf("=== SIM PERFORMANCE ===\n");
    printf("wall-clock budget:    %.3f s\n", budget_s);
    printf("wall-clock measured:  %.3f s\n", wall);
    printf("ticks computed:       %ld\n", ticks);
    printf("ticks per second:     %.0f\n", ticks_ps);
    printf("simulated seconds:    %.2f s\n", sim_s);
    printf("real-time factor:     %.0fx  (sim runs %.0f times faster than real time)\n", rt_factor,
        rt_factor);
    printf("laps completed:       %d\n", laps);
    printf("laps per wall-second: %.1f\n", laps_ps);

    // machine-readable line for scripts and CI
    printf("PERF ticks=%ld ticks_per_s=%.0f sim_s=%.2f rt_factor=%.1f "
           "laps=%d laps_per_s=%.2f wall_s=%.3f\n",
        ticks, ticks_ps, sim_s, rt_factor, laps, laps_ps, wall);
    return 0;
}
