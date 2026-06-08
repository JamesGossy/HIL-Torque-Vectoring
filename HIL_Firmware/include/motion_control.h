#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "track.h"
#include "vehicle_model.h"
#include "../../shared/parameters_config.h"

/*
 * motion_control.h
 *
 * The "driver" layer. Runs every tick and produces a steering angle (written
 * into state->steering) and a driver torque demand (the return value, Nm;
 * positive is throttle, negative is regen braking).
 *
 * Steering is Pure Pursuit: it aims at a point on the racing line a look-ahead
 * distance ahead of the rear axle and computes the single-arc steer angle that
 * drives the rear axle through it. A short look-ahead at low speed commits the
 * car to a tight apex; a longer one at speed keeps the line smooth. A small
 * cross-track trim pulls the car back when it drifts off the line, and a cone
 * repulsion term is a safety net near the boundary.
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

#endif /* MOTION_CONTROL_H */
