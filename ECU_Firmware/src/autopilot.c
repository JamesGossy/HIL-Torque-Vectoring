// Top-level autonomous ECU: SLAM -> online planner -> motion control -> torque
// vectoring, all from SensorData. Large state (SlamState, EcuMap) is file-static
// to keep it off the stack.

#include "../include/autopilot.h"
#include "../include/slam.h"
#include "../include/online_planner.h"
#include "../include/motion_control.h"
#include "../include/ecu_map.h"
#include "../include/torque_vectoring.h"
#include "../../shared/tunables.h"
#include <math.h>

static SlamState s_slam;
static OnlinePlanner s_planner;
static EcuMap s_map;
static float s_ay_est;     // lagged lateral-accel estimate (ECU has no true ay)
static float s_last_steer; // last applied steering, for the slew limit

void autopilot_init(float start_x, float start_y, float start_heading)
{
    slam_init(&s_slam, start_x, start_y, start_heading);
    online_planner_init(&s_planner);
    motion_control_reset();
    torque_vectoring_reset();
    s_ay_est     = 0.0f;
    s_last_steer = 0.0f;
}

void autopilot_get_pose(float *x, float *y, float *heading)
{
    slam_get_pose(&s_slam, x, y, heading);
}

int autopilot_landmark_count(void)
{
    return s_slam.n_land;
}

const SlamState *autopilot_slam(void)
{
    return &s_slam;
}

void autopilot_update(const SensorData *sensors, float dt, DriveCommand *cmd)
{
    // 1. SLAM: predict from odometry, update on cone observations.
    slam_predict(&s_slam, sensors, dt);
    slam_update(&s_slam, &sensors->scan);

    float ex, ey, eh;
    slam_get_pose(&s_slam, &ex, &ey, &eh);

    // 2. Planner: build the line to follow this tick from the SLAM map.
    float speed_cap = online_planner_step(&s_planner, &s_slam, ex, ey, eh, &s_map);

    s_ay_est += 0.1f * (sensors->velocity * sensors->yaw_rate - s_ay_est);

    float steer, driver_torque;
    if (s_planner.phase2_active) {
        // 3a. Phase 2: race the optimised line with the full Stanley controller.
        CtrlPose pose
            = { ex, ey, eh, sensors->velocity, sensors->yaw_rate, s_ay_est, s_last_steer };
        float target  = 0.0f;
        driver_torque = motion_control_update(&pose, &s_map, &steer, &target);
    } else {
        // 3b. Phase 1: pure-pursuit toward a lookahead gate midpoint. Far less
        // sensitive to the jittery per-tick local line than Stanley, which keys
        // off one short segment. Lookahead is short so it does not overshoot the
        // apex on tight corners (a long Ld under-steers and runs wide).
        float Ld    = 3.0f + 0.5f * sensors->velocity;
        float alpha = 0.0f;
        int have_target = 0;
        float tx, ty;
        if (online_planner_lookahead(&s_map, ex, ey, Ld, &tx, &ty)) {
            alpha = atan2f(ty - ey, tx - ex) - eh;
            while (alpha > 3.14159265f)
                alpha -= 6.2831853f;
            while (alpha < -3.14159265f)
                alpha += 6.2831853f;
            float ld_act = sqrtf((tx - ex) * (tx - ex) + (ty - ey) * (ty - ey));
            if (ld_act < 1.0f) ld_act = 1.0f;
            float delta_wheel = atan2f(2.0f * WHEELBASE_M * sinf(alpha), ld_act);
            steer             = delta_wheel / ACK_NOMINAL;
            have_target       = 1;
        } else {
            steer = s_last_steer; // no target yet, hold
        }
        // clamp + slew like the normal controller
        if (steer > g_MAX_STEER_RAD) steer = g_MAX_STEER_RAD;
        if (steer < -g_MAX_STEER_RAD) steer = -g_MAX_STEER_RAD;
        float max_step = g_MAX_STEER_RATE_RADS * dt;
        float dsteer   = steer - s_last_steer;
        if (dsteer > max_step) steer = s_last_steer + max_step;
        if (dsteer < -max_step) steer = s_last_steer - max_step;

        // Corner slowdown: the sharper the heading we must take to reach the
        // lookahead point, the slower we go, so pure pursuit does not run wide.
        float cap = speed_cap;
        if (have_target) {
            float turn = fabsf(alpha);
            cap        = speed_cap / (1.0f + 2.5f * turn); // turn~0.6 rad -> ~40% speed
            if (cap < 2.5f) cap = 2.5f;
        }
        float verr    = cap - sensors->velocity;
        driver_torque = (verr >= 0.0f) ? g_SPEED_KP_NM * verr : g_BRAKE_KP_NM * verr;
        if (driver_torque > DRIVER_TORQUE_NM) driver_torque = DRIVER_TORQUE_NM;
        if (driver_torque < DRIVER_BRAKE_NM) driver_torque = DRIVER_BRAKE_NM;
    }

    s_last_steer = steer;

    // 4. Torque vectoring splits the demand across the wheels.
    torque_vectoring_update(sensors, driver_torque, g_KP_YAW, &cmd->torques);
    cmd->steering_rad = steer;
}
