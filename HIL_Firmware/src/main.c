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
#include "../include/track_parser.h"
#include "../include/motion_control.h"
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
 *     STATE <x> <y> <heading> <speed_kmh> <yaw_deg_s> <fl> <fr> <rl> <rr> <tv> <kp> <lap>
 * <elapsed_s> <steering_rad> <slip_angle_rad> <desired_yaw_rad_s> <ax_ms2> <ay_ms2> <vy_ms>
 * <target_speed_kmh>
 *
 *   Commands come in on stdin (one character at a time):
 *     t  -- toggle torque vectoring
 *     [  -- decrease Kp by 5
 *     ]  -- increase Kp by 5
 *     q  -- quit
 */

#define SIM_HZ          100
#define DISPLAY_HZ      20
#define DT              (1.0f / SIM_HZ)
#define TICKS_PER_FRAME (SIM_HZ / DISPLAY_HZ)


/* ---- Platform-specific non-blocking stdin read ---- */

#ifdef _WIN32

static void stdin_setup(void)
{ /* nothing needed */
}
static void stdin_restore(void)
{ /* nothing needed */
}

static int poll_stdin(void)
{
    HANDLE h    = GetStdHandle(STD_INPUT_HANDLE);
    DWORD avail = 0;
    if (!PeekNamedPipe(h, NULL, 0, NULL, &avail, NULL) || avail == 0) return -1;
    DWORD read_count = 0;
    unsigned char c;
    if (ReadFile(h, &c, 1, &read_count, NULL) && read_count == 1) return (int)c;
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
    static LARGE_INTEGER freq = { 0 };
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    LARGE_INTEGER count;
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
    Track track;
    VehicleState state;
    WheelTorques torques = { 0 };
    SensorData sensors   = { 0 };

    track_init(&track);

    if (track.count < 2) {
        fprintf(stderr, "track_init produced fewer than 2 waypoints - cannot run\n");
        return 1;
    }

    /* Derive initial heading from the direction of the first track segment */
    float init_dx      = track.points[1].x - track.points[0].x;
    float init_dy      = track.points[1].y - track.points[0].y;
    float init_heading = atan2f(init_dy, init_dx);
    vehicle_model_init(&state, track.points[0].x, track.points[0].y, init_heading);

    /* Start every controller from a clean slate (driver progress + throttle
     * integrator + LQR steering state, and the ECU's yaw PID), so no state
     * leaks in from a stale static into this run. */
    motion_control_reset();
    torque_vectoring_reset();

    /* --- Print track data so the visualiser can draw the track and cones --- */
    printf("TRACK %d\n", track.count);
    for (int i = 0; i < track.count; i++) {
        printf("WP %.4f %.4f\n", track.points[i].x, track.points[i].y);
    }
    printf("END_TRACK\n");

    printf("LEFT_CONES %d\n", track.left_count);
    for (int i = 0; i < track.left_count; i++) {
        printf("CONE %.4f %.4f\n", track.left_cones[i].x, track.left_cones[i].y);
    }
    printf("END_LEFT_CONES\n");

    printf("RIGHT_CONES %d\n", track.right_count);
    for (int i = 0; i < track.right_count; i++) {
        printf("CONE %.4f %.4f\n", track.right_cones[i].x, track.right_cones[i].y);
    }
    printf("END_RIGHT_CONES\n");

    fflush(stdout);

    stdin_setup();

    int tv_enabled   = 1;
    float kp_yaw     = KP_YAW_DEFAULT;
    int tick         = 0;
    int running      = 1;
    double sim_start = get_time_s();
    double next_tick = sim_start;

    while (running) {
        double now = get_time_s();

        if (now < next_tick) {
            sleep_ms(1);
            continue;
        }
        next_tick += DT;
        tick++;

        /* 1. Motion control */
        float target_speed  = 0.0f;
        float driver_torque = motion_control_update(&state, &track, &target_speed);

        /* 2. Pack sensor data */
        sensors.yaw_rate       = state.yaw_rate;
        sensors.velocity       = state.velocity;
        sensors.steering_angle = state.steering;
        sensors.driver_torque  = driver_torque;

        /* Real per-corner wheel speeds from the vehicle model.  The model
         * stores motor-shaft RPM; convert to wheel angular speed (rad/s) for
         * the sensor bus: wheel_rad/s = motor_RPM / GEAR_RATIO * 2π/60. */
        const float RPM_TO_WHEEL_RADS = (2.0f * 3.14159265358979f) / (GEAR_RATIO * 60.0f);
        sensors.wheel_speed[WHEEL_FL] = state.wheelspeed[WHEEL_FL] * RPM_TO_WHEEL_RADS;
        sensors.wheel_speed[WHEEL_FR] = state.wheelspeed[WHEEL_FR] * RPM_TO_WHEEL_RADS;
        sensors.wheel_speed[WHEEL_RL] = state.wheelspeed[WHEEL_RL] * RPM_TO_WHEEL_RADS;
        sensors.wheel_speed[WHEEL_RR] = state.wheelspeed[WHEEL_RR] * RPM_TO_WHEEL_RADS;

        /* 3. ECU */
        if (tv_enabled) {
            torque_vectoring_update(&sensors, driver_torque, kp_yaw, &torques);
        } else {
            /* TV off: split the driver's MOTOR-torque demand evenly across the
             * four motors.  The vehicle model applies GEAR_RATIO itself, so we
             * must not multiply by it here (that double-counts the ratio). */
            float base = driver_torque * 0.25f;
            torques.fl = torques.fr = torques.rl = torques.rr = base;
        }

        /* 4. Vehicle model */
        vehicle_model_update(&state, &torques, DT);

        /* 5. Track */
        track_update(&track, state.x, state.y);

        /* 6. Print state line at display rate */
        if (tick % TICKS_PER_FRAME == 0) {
            float elapsed          = (float)(get_time_s() - sim_start);
            float desired_yaw_rate = 0.0f;
            if (state.velocity > 0.5f)
                desired_yaw_rate = state.velocity * tanf(state.steering) / WHEELBASE_M;
            printf("STATE %.3f %.3f %.4f %.2f %.2f %.1f %.1f %.1f %.1f %d %.1f %d %.1f %.4f %.4f "
                   "%.4f %.3f %.3f %.3f %.2f\n",
                state.x, state.y, state.heading, state.velocity * 3.6f, /* m/s -> km/h */
                state.yaw_rate * 57.2958f,                              /* rad/s -> deg/s */
                torques.fl, torques.fr, torques.rl, torques.rr, tv_enabled, kp_yaw, track.lap_count,
                elapsed, state.steering, /* radians */
                state.slip_angle,        /* radians */
                desired_yaw_rate,        /* rad/s */
                state.ax,                /* m/s^2 (for G-G display) */
                state.ay,                /* m/s^2 (for G-G display) */
                state.vy,                /* m/s  (lateral velocity) */
                target_speed * 3.6f);    /* planner target speed, km/h */
            fflush(stdout);
        }

        /* 7. Commands from stdin */
        int cmd = poll_stdin();
        if (cmd != -1) {
            switch (cmd) {
            case 't':
            case 'T':
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
            case 'q':
            case 'Q':
            case 27:
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
