#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "track.h"
#include "vehicle_model.h"

/*
 * autopilot.h
 *
 * The autopilot is the "virtual driver". It looks at where the car is and
 * where the next track waypoint is, and decides what steering angle to use.
 *
 * This is a pure-pursuit controller. The idea is simple:
 *   - Pick a target point some distance ahead on the track (the "lookahead" point).
 *   - Calculate the angle from the car to that point.
 *   - Steer toward that angle.
 *
 * It is called "pure pursuit" because the car purely pursues the lookahead point
 * with no prediction or model of the car dynamics.
 *
 * The autopilot is intentionally separate from the vehicle model and the ECU code.
 * In a real HIL test, the "driver" could be:
 *   - A human turning a steering wheel
 *   - A pre-recorded driver input file
 *   - This autopilot
 * Keeping it separate means you can swap it out without touching the physics or ECU.
 */


/* How far ahead on the track the autopilot looks, in metres.
 * Longer lookahead = smoother steering but cuts corners more.
 * Shorter lookahead = tighter lines but can feel nervous and oscillate. */
#define LOOKAHEAD_M  12.0f

/* A target speed the autopilot tries to maintain, m/s.
 * The actual speed depends on the motor torque from the ECU. This is the
 * speed the autopilot requests via the driver_torque demand. */
#define TARGET_SPEED_MS  22.0f

/* Maximum torque the driver can request, Nm total across all four wheels. */
#define DRIVER_TORQUE_NM  800.0f

/* P-controller gain for speed: Nm of torque demand per m/s of speed error. */
#define SPEED_KP_NM       60.0f

/* Feedforward for drag: Nm total per m/s of velocity.
 * = DRAG_COEFF * WHEEL_RADIUS_M = 20.0 * 0.25  (must match vehicle_model.h) */
#define DRAG_FF_NM         5.0f


/*
 * Run the autopilot for one tick.
 *
 * Reads the car's current position and heading from the vehicle state.
 * Reads the track waypoints to find the lookahead target.
 * Writes the computed steering angle back into state->steering.
 * Returns the driver torque demand (total Nm, to be split by the ECU).
 */
float autopilot_update(VehicleState *state, const Track *track);

#endif /* AUTOPILOT_H */
