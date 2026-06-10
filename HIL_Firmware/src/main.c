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
#include "../../shared/tunables.h"
#include "../../ECU_Firmware/include/torque_vectoring.h"

/*
 * main.c
 *
 * The simulation loop. The driver, car, and ECU talk to each other each tick.
 * On startup it prints the track for the visualiser, then streams one CSV
 * state line per display tick. Single-character commands arrive on stdin.
 */

#define SIM_HZ          100
#define DISPLAY_HZ      20
#define DT              (1.0f / SIM_HZ)
#define TICKS_PER_FRAME (SIM_HZ / DISPLAY_HZ)


/* ---- platform: raw stdin and timing ---- */

#ifdef _WIN32

// Put stdin into raw non-blocking mode (no-op on Windows).
static void stdin_setup(void)
{
}
// Restore stdin to its original mode (no-op on Windows).
static void stdin_restore(void)
{
}

// Read one pending stdin byte, or return -1 if none.
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

// Sleep for the given milliseconds.
static void sleep_ms(int ms)
{
    Sleep(ms);
}

#else

static struct termios g_old_termios;

// Put stdin into raw non-blocking mode.
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

// Restore stdin to its original mode.
static void stdin_restore(void)
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
}

// Read one pending stdin byte, or return -1 if none.
static int poll_stdin(void)
{
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) == 1) return (int)c;
    return -1;
}

// Sleep for the given milliseconds.
static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

#endif


// Return a monotonic time in seconds.
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


// Run the driver-ECU-vehicle simulation loop until quit.
int main(void)
{
    Track track;
    VehicleState state;
    WheelTorques torques = { 0 };
    SensorData sensors   = { 0 };

    // load tunables and build the track
    tunables_init_from_env();

    track_init(&track);

    if (track.count < 2) {
        fprintf(stderr, "track_init produced fewer than 2 waypoints - cannot run\n");
        return 1;
    }

    // place the car at the start, facing the first segment
    float init_dx      = track.points[1].x - track.points[0].x; // heading from first segment
    float init_dy      = track.points[1].y - track.points[0].y;
    float init_heading = atan2f(init_dy, init_dx);
    vehicle_model_init(&state, track.points[0].x, track.points[0].y, init_heading);

    motion_control_reset();    // clear stale static state from any prior run
    torque_vectoring_reset();

    printf("TRACK %d\n", track.count); // print track and cones for the visualiser
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

    // loop state and pacing clock
    int tv_enabled   = 1;
    float kp_yaw     = g_KP_YAW;
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

        float target_speed  = 0.0f; // motion control
        float driver_torque = motion_control_update(&state, &track, &target_speed);

        sensors.yaw_rate       = state.yaw_rate; // pack sensor data
        sensors.velocity       = state.velocity;
        sensors.steering_angle = state.steering;
        sensors.driver_torque  = driver_torque;

        // convert motor-shaft RPM to wheel rad/s for the sensor bus
        const float RPM_TO_WHEEL_RADS = (2.0f * 3.14159265358979f) / (GEAR_RATIO * 60.0f);
        sensors.wheel_speed[WHEEL_FL] = state.wheelspeed[WHEEL_FL] * RPM_TO_WHEEL_RADS;
        sensors.wheel_speed[WHEEL_FR] = state.wheelspeed[WHEEL_FR] * RPM_TO_WHEEL_RADS;
        sensors.wheel_speed[WHEEL_RL] = state.wheelspeed[WHEEL_RL] * RPM_TO_WHEEL_RADS;
        sensors.wheel_speed[WHEEL_RR] = state.wheelspeed[WHEEL_RR] * RPM_TO_WHEEL_RADS;

        if (tv_enabled) { // ECU
            torque_vectoring_update(&sensors, driver_torque, kp_yaw, &torques);
        } else {
            float base = driver_torque * 0.25f; // even motor split, model applies GEAR_RATIO so do not multiply here
            torques.fl = torques.fr = torques.rl = torques.rr = base;
        }

        vehicle_model_update(&state, &torques, DT); // advance physics

        track_update(&track, state.x, state.y); // check waypoint progress

        if (tick % TICKS_PER_FRAME == 0) { // print state line at display rate
            float elapsed          = (float)(get_time_s() - sim_start);
            float desired_yaw_rate = 0.0f;
            if (state.velocity > 0.5f)
                desired_yaw_rate = state.velocity * tanf(state.steering) / WHEELBASE_M;
            printf("STATE %.3f %.3f %.4f %.2f %.2f %.1f %.1f %.1f %.1f %d %.1f %d %.1f %.4f %.4f "
                   "%.4f %.3f %.3f %.3f %.2f\n",
                state.x, state.y, state.heading, state.velocity * 3.6f, // km/h
                state.yaw_rate * 57.2958f,                              // deg/s
                torques.fl, torques.fr, torques.rl, torques.rr, tv_enabled, kp_yaw, track.lap_count,
                elapsed, state.steering, state.slip_angle, desired_yaw_rate, state.ax, state.ay,
                state.vy, target_speed * 3.6f);
            fflush(stdout);
        }

        int cmd = poll_stdin(); // commands from stdin

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
