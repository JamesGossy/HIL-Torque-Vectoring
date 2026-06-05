#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifdef _WIN32
    #include <windows.h>
    #include <conio.h>
#else
    #include <unistd.h>
    #include <termios.h>
    #include <fcntl.h>
#endif

#include "../include/vehicle_model.h"
#include "../include/track.h"
#include "../include/autopilot.h"
#include "../../shared/tv_interface.h"
#include "../../ECU_Firmware/include/torque_vectoring.h"

/*
 * main.c
 *
 * The simulation loop. This is where the "Driver", "Car", and "ECU" talk to
 * each other every tick.
 *
 * Each tick (10 ms = 100 Hz):
 *   1. Autopilot (driver)    -> sets steering angle, returns driver torque demand
 *   2. ECU firmware          -> splits driver torque to four wheel torques
 *   3. Vehicle model (car)   -> advances the physics by dt seconds
 *   4. Track                 -> checks if the car has reached the next waypoint
 *   5. State output          -> prints a CSV line to stdout (at 20 Hz)
 *
 * Output protocol:
 *   On startup, this program prints the track waypoints so the visualiser can
 *   draw the track. Then it streams one CSV line per display tick.
 *
 *   Header block (one line per waypoint):
 *     TRACK <count>
 *     WP <x> <y>
 *     WP <x> <y>
 *     ...
 *     END_TRACK
 *
 *   State lines (one per display tick):
 *     STATE <x> <y> <heading> <speed_kmh> <yaw_deg_s> <fl> <fr> <rl> <rr> <tv> <kp> <lap> <elapsed_s>
 *
 *   Commands come in on stdin (one character at a time):
 *     t  -- toggle torque vectoring
 *     [  -- decrease Kp by 5
 *     ]  -- increase Kp by 5
 *     q  -- quit
 */

#define SIM_HZ           100
#define DISPLAY_HZ        20
#define DT               (1.0f / SIM_HZ)
#define TICKS_PER_FRAME  (SIM_HZ / DISPLAY_HZ)


/* ---- Platform-specific non-blocking stdin read ---- */

#ifdef _WIN32

static void stdin_setup(void)   { /* nothing needed */ }
static void stdin_restore(void) { /* nothing needed */ }

static int poll_stdin(void)
{
    HANDLE h = GetStdHandle(STD_INPUT_HANDLE);
    DWORD  avail = 0;
    if (!PeekNamedPipe(h, NULL, 0, NULL, &avail, NULL) || avail == 0)
        return -1;
    DWORD read_count = 0;
    unsigned char c;
    if (ReadFile(h, &c, 1, &read_count, NULL) && read_count == 1)
        return (int)c;
    return -1;
}

static void sleep_ms(int ms)
{
    Sleep(ms);
}

#else

static struct termios g_old_termios;

static void stdin_setup(void)
{
    struct termios raw;
    tcgetattr(STDIN_FILENO, &g_old_termios);
    raw = g_old_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

static void stdin_restore(void)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
}

static int poll_stdin(void)
{
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) == 1) return (int)c;
    return -1;
}

static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

#endif


/* ---- Timing ---- */

static double get_time_s(void)
{
#ifdef _WIN32
    LARGE_INTEGER freq, count;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return (double)count.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
#endif
}


/* ---- Main ---- */

int main(void)
{
    Track        track;
    VehicleState state;
    WheelTorques torques = {0};
    SensorData   sensors = {0};

    track_init(&track);
    /* Start heading = pi/2 (north): the right loop leaves the crossing point
     * travelling upward (CCW), so the initial heading must match that direction. */
    vehicle_model_init(&state, track.points[0].x, track.points[0].y, 3.14159265f / 2.0f);

    /* --- Print track waypoints so the visualiser can draw the track --- */
    printf("TRACK %d\n", track.count);
    for (int i = 0; i < track.count; i++) {
        printf("WP %.4f %.4f\n", track.points[i].x, track.points[i].y);
    }
    printf("END_TRACK\n");
    fflush(stdout);

    stdin_setup();

    int    tv_enabled = 1;
    float  kp_yaw     = KP_YAW_DEFAULT;
    int    tick       = 0;
    int    running    = 1;
    double sim_start  = get_time_s();
    double next_tick  = sim_start;

    while (running)
    {
        double now = get_time_s();

        if (now < next_tick) {
            sleep_ms(1);
            continue;
        }
        next_tick += DT;
        tick++;

        /* 1. Autopilot */
        float driver_torque = autopilot_update(&state, &track);

        /* 2. Pack sensor data */
        sensors.yaw_rate       = state.yaw_rate;
        sensors.velocity       = state.velocity;
        sensors.steering_angle = state.steering;
        sensors.driver_torque  = driver_torque;

        float wheel_speed_rads = state.velocity / WHEEL_RADIUS_M;
        sensors.wheel_speed[WHEEL_FL] = wheel_speed_rads;
        sensors.wheel_speed[WHEEL_FR] = wheel_speed_rads;
        sensors.wheel_speed[WHEEL_RL] = wheel_speed_rads;
        sensors.wheel_speed[WHEEL_RR] = wheel_speed_rads;

        /* 3. ECU */
        if (tv_enabled) {
            torque_vectoring_update(&sensors, driver_torque, kp_yaw, &torques);
        } else {
            float base = driver_torque * 0.25f;
            torques.fl = torques.fr = torques.rl = torques.rr = base;
        }

        /* 4. Vehicle model */
        vehicle_model_update(&state, &torques, DT);

        /* 5. Track */
        track_update(&track, state.x, state.y);

        /* 6. Print state line at display rate */
        if (tick % TICKS_PER_FRAME == 0) {
            float elapsed = (float)(get_time_s() - sim_start);
            printf("STATE %.3f %.3f %.4f %.2f %.2f %.1f %.1f %.1f %.1f %d %.1f %d %.1f %.4f\n",
                   state.x,
                   state.y,
                   state.heading,
                   state.velocity * 3.6f,       /* m/s -> km/h */
                   state.yaw_rate * 57.2958f,   /* rad/s -> deg/s */
                   torques.fl, torques.fr, torques.rl, torques.rr,
                   tv_enabled,
                   kp_yaw,
                   track.lap_count,
                   elapsed,
                   state.steering);             /* steering angle, radians */
            fflush(stdout);
        }

        /* 7. Commands from stdin */
        int cmd = poll_stdin();
        if (cmd != -1) {
            switch (cmd) {
                case 't': case 'T':
                    tv_enabled = !tv_enabled;
                    break;
                case '[':
                    kp_yaw -= 5.0f;
                    if (kp_yaw < 0.0f) kp_yaw = 0.0f;
                    break;
                case ']':
                    kp_yaw += 5.0f;
                    if (kp_yaw > 500.0f) kp_yaw = 500.0f;
                    break;
                case 'q': case 'Q': case 27:
                    running = 0;
                    break;
                default:
                    break;
            }
        }
    }

    stdin_restore();
    return 0;
}
