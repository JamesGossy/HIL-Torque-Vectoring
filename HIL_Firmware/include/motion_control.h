#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "track_parser.h"
#include "vehicle_model.h"
#include "../../shared/parameters_config.h"

/*
 * motion_control.h
 *
 * The "driver" layer. Runs every tick and produces a steering angle (written
 * into state->steering) and a driver torque demand (the return value, Nm;
 * positive is throttle, negative is regen braking).
 *
 * Steering is a model-based LQR law (see lqr_steer.h): the front axle is
 * projected onto the racing line to read the cross-track and heading error, and
 * an optimal feedback law on the dynamic-bicycle error dynamics computes the
 * steer command. A cone-repulsion term is a safety net near the boundary.
 *
 * Speed is a two-pass planner: a forward pass caps each waypoint's speed from
 * its curvature, then a backward pass propagates braking from the furthest
 * waypoint back to the car. Throttle is faded out near full steering lock,
 * where the front tyres are saturated turning and extra power only pushes wide.
 *
 * All tunable gains live in shared/parameters_config.h.
 */

/*
 * Run one motion-control tick. Writes state->steering and returns the driver
 * torque demand. out_target_speed, if non-NULL, receives the planner's target
 * speed for this tick (telemetry only).
 */
float motion_control_update(VehicleState *state, const Track *track,
                            float *out_target_speed);

/*
 * Reset the driver's internal state (progress index, throttle integrator) and
 * the LQR steering state it owns. Call before an independent run so no state
 * leaks in from a previous one; the sim entry points do this after track_init().
 */
void motion_control_reset(void);

#endif /* MOTION_CONTROL_H */
