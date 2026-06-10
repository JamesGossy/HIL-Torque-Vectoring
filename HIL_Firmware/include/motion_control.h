#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "track_parser.h"
#include "vehicle_model.h"
#include "../../shared/vehicle_config.h"
#include "../../shared/tunables.h"

/*
 * The driver layer. Each tick it writes a steering angle into state->steering
 * and returns a driver torque demand in Nm (positive throttle, negative regen).
 * Steering is Stanley feedback plus a curvature feedforward; speed is a
 * two-pass planner under the friction circle. Gains live in shared/tunables.c,
 * physical limits in shared/vehicle_config.h.
 */

/* ---- driver lifecycle ---- */

// Run one tick. Writes state->steering and returns torque demand; out_target_speed gets the planner target if non-NULL.
float motion_control_update(VehicleState *state, const Track *track, float *out_target_speed);

// Reset driver state (progress index, throttle integrator) before an independent run.
void motion_control_reset(void);

/* ---- steering and tyre model ---- */

// Steering reference (rad) from cross-track e1 (+ve left of line), heading error e2, yaw_rate, path_kappa, speed vx.
float steer_command(float vx, float e1, float e2, float yaw_rate, float path_kappa);

// Understeer gradient (rad per m/s^2) derived from the tyre model.
float driver_understeer_gradient(void);

#endif /* MOTION_CONTROL_H */
