#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "track_parser.h"
#include "vehicle_model.h"
#include "../../shared/vehicle_config.h"
#include "../../shared/tunables.h"

/*
 * motion_control.h
 *
 * The "driver" layer. Runs every tick and produces a steering angle (written
 * into state->steering) and a driver torque demand (the return value, Nm;
 * positive is throttle, negative is regen braking).
 *
 * Steering is a Stanley law plus a physics-derived curvature feedforward (see
 * steer_command below): the front axle is projected onto the racing line to read
 * the cross-track and heading error, the feedforward holds the path radius from
 * the tyre/geometry constants, and the Stanley feedback pulls the car onto the
 * line with one gain (g_K_STANLEY). A cone-repulsion term is a safety net near
 * the boundary.
 *
 * Speed is a two-pass planner: a forward pass caps each waypoint's speed from
 * its curvature (at g_GRIP_USE of the physical peak grip, shared/grip_model.h),
 * then a backward pass propagates braking from the furthest waypoint back to the
 * car under the friction circle. Throttle is faded out near full steering lock,
 * where the front tyres are saturated turning and extra power only pushes wide.
 *
 * Every gain is a g_* global in shared/tunables.c; the car's physical and
 * actuator limits are in shared/vehicle_config.h. There are only those two files
 * of numbers - the old shared/constants_config.h is gone.
 */

/*
 * Run one motion-control tick. Writes state->steering and returns the driver
 * torque demand. out_target_speed, if non-NULL, receives the planner's target
 * speed for this tick (telemetry only).
 */
float motion_control_update(VehicleState *state, const Track *track, float *out_target_speed);

/*
 * Reset the driver's internal state (progress index, throttle integrator). Call
 * before an independent run so no state leaks in from a previous one; the sim
 * entry points do this after track_init(). The Stanley steering law keeps no
 * integrator, so there is no steering state to clear.
 */
void motion_control_reset(void);

/*
 * Steering law (kinematic curvature feedforward + Stanley feedback with
 * yaw-rate damping): returns the steering reference (rad) for cross-track error
 * e1 (+ve when the car is LEFT of the line), heading error e2 (yaw - path
 * heading), measured yaw_rate, and signed path curvature path_kappa (+ve for a
 * left-hand bend), at speed vx. The vehicle model scales the reference by the
 * Ackermann ratio. Exposed for unit tests.
 */
float steer_command(float vx, float e1, float e2, float yaw_rate, float path_kappa);

/* Understeer gradient (rad per m/s^2 lateral accel) derived from the tyre model;
 * used by the steering feedforward. Exposed for tests. */
float driver_understeer_gradient(void);

#endif /* MOTION_CONTROL_H */
