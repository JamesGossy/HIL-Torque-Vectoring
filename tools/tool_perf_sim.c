/*
 * tools/perf_sim.c
 *
 * Compute-speed benchmark (not a unit test). Runs the main.c tick loop with no
 * real-time sleep for a fixed wall-clock time (default 1 s) and reports
 * throughput: ticks computed, real-time factor, and laps per wall-second. It
 * measures host compute speed, not driving quality. Build it with `make perf`.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../HIL_Firmware/include/vehicle_model.h"
#include "../HIL_Firmware/include/track_parser.h"
#include "../HIL_Firmware/include/motion_control.h"
#include "../shared/tv_interface.h"
#include "../ECU_Firmware/include/torque_vectoring.h"

#define DT 0.01f /* simulated seconds per tick (100 Hz), matches main.c */

/* ---- Monotonic wall-clock, same pattern as main.c ---- */
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

int main(int argc, char **argv)
{
    /* Wall-clock budget in seconds (default 1.0; override with argv[1]). */
    double budget_s = 1.0;
    if (argc > 1) {
        double v = atof(argv[1]);
        if (v > 0.0) budget_s = v;
    }

    Track track;
    VehicleState state;
    WheelTorques torques = { 0 };
    SensorData sensors   = { 0 };

    track_init(&track);

    float ih = atan2f(track.points[1].y - track.points[0].y, track.points[1].x - track.points[0].x);
    vehicle_model_init(&state, track.points[0].x, track.points[0].y, ih);

    /* Clean controller state for this run (driver + LQR steering + ECU yaw PID). */
    motion_control_reset();
    torque_vectoring_reset();

    const float R2W = (2.0f * 3.14159265358979f) / (GEAR_RATIO * 60.0f);
    int start_lap   = track.lap_count;
    long ticks      = 0;

    /* Check the clock only every CHECK_EVERY ticks so the timing call itself
     * does not dominate the measured loop (and so we count whole batches). */
    const long CHECK_EVERY = 256;

    double t0  = wall_s();
    double now = t0;
    for (;;) {
        for (long b = 0; b < CHECK_EVERY; b++) {
            float target_speed = 0.0f;
            float dq           = motion_control_update(&state, &track, &target_speed);

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

    /* Machine-readable line for scripts / CI. */
    printf("PERF ticks=%ld ticks_per_s=%.0f sim_s=%.2f rt_factor=%.1f "
           "laps=%d laps_per_s=%.2f wall_s=%.3f\n",
        ticks, ticks_ps, sim_s, rt_factor, laps, laps_ps, wall);
    return 0;
}
