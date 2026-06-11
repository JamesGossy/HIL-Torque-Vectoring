#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "ecu_map.h"
#include "../../shared/vehicle_config.h"
#include "../../shared/tunables.h"

/*
 * The driver layer, now on the ECU. Each tick it computes a steering angle and a
 * driver torque demand from an estimated pose and the ECU's own map. Steering is
 * Stanley feedback plus a curvature feedforward; speed is a two-pass planner
 * under the friction circle. Gains live in shared/tunables.c, physical limits in
 * shared/vehicle_config.h.
 */

/* ---- types ---- */

// Pose the controller drives on. From SLAM in autonomy, from ground truth in the legacy path.
typedef struct {
    float x;
    float y;
    float heading;
    float vx;
    float yaw_rate;
    float ay_filt;  // lagged lateral accel for the traction cut
    float steering; // last applied steering, for the slew-rate limit
} CtrlPose;

/* ---- driver lifecycle ---- */

// Run one tick. Writes the steering angle to *out_steering_rad and returns torque demand; out_target_speed gets the planner target if non-NULL.
float motion_control_update(
    const CtrlPose *pose, const EcuMap *map, float *out_steering_rad, float *out_target_speed);

// Reset driver state (progress index, throttle integrator) before an independent run.
void motion_control_reset(void);

/* ---- steering and tyre model ---- */

// Steering reference (rad) from cross-track e1 (+ve left of line), heading error e2, yaw_rate, path_kappa, speed vx.
float steer_command(float vx, float e1, float e2, float yaw_rate, float path_kappa);

// Understeer gradient (rad per m/s^2) derived from the tyre model.
float driver_understeer_gradient(void);

#endif /* MOTION_CONTROL_H */
