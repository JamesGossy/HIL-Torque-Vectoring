#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include "../../shared/tv_interface.h"

/*
 * torque_vectoring.h
 *
 * This is the ECU side of the HIL system. In real life, this code would run
 * on an embedded microcontroller. Here, it is compiled into the same binary
 * as the HIL simulation, but it only ever sees SensorData -- not the full
 * vehicle model state. That is the HIL/ECU boundary.
 *
 * The torque vectoring algorithm does one thing:
 *   Take a total torque demand from the driver and decide how to split it
 *   between the four wheels to help the car follow the intended path.
 *
 * The approach is yaw-rate control:
 *   - Compute the yaw rate the car *should* have (from steering angle and speed).
 *   - Compare that to the yaw rate it *actually* has (from the IMU).
 *   - If there is an error, move torque from the inner wheels to the outer wheels
 *     to help the car rotate more (or less) as needed.
 *
 * Runtime tuning (no recompile needed):
 *   Press [ or ] in the terminal to decrease or increase KP_YAW by 5.
 *   Press t to toggle the whole TV system on or off.
 */


/* Proportional gain: how aggressively to respond to yaw rate error.
 * Units: Nm of torque bias per rad/s of yaw rate error.
 * Start here and adjust with [ / ] at runtime.
 * The higher the gain, the harder TV fights to hit the desired yaw rate.
 * Units are Nm (motor) of bias per rad/s of yaw error; the bias is clamped to
 * half the motor peak (~14.7 Nm), so a gain that drives typical errors of a
 * few tenths of a rad/s past that clamp just saturates and bangs between the
 * rails.  60 keeps the differential proportional through normal cornering;
 * raise it with ] until the bias starts to saturate, then back off. */
#define KP_YAW_DEFAULT  60.0f

/* Yaw-rate error deadband, rad/s.  Errors smaller than this are treated as
 * zero so the torque differential does not chatter about zero on sensor noise
 * and the inevitable steady-state bias of the kinematic desired-yaw estimate. */
#define TV_YAW_DEADBAND  0.03f

/* Reference speed for speed-dependent gain scaling.
 * Gain = Kp * (TV_SPEED_REF_MS / v).  Set to 12 so TV stays authoritative
 * up through corner entry speeds (typically 10–15 m/s). */
#define TV_SPEED_REF_MS  12.0f

/* Hard limit on torque per motor, Nm. Represents motor peak torque. */
#define MAX_MOTOR_TORQUE_NM   29.4f

/* Minimum motor torque (negative = regenerative braking). */
#define MIN_MOTOR_TORQUE_NM  -100.0f

/*
 * Compute wheel torques from sensor data.
 *
 * sensors       -- what the ECU can sense (yaw rate, speed, steering, wheel speeds)
 * driver_torque -- total torque demanded by the driver, Nm
 * kp_yaw        -- proportional gain (can be changed at runtime by the user)
 * out           -- filled with the four wheel torque demands
 */
void torque_vectoring_update(const SensorData *sensors,
                             float             driver_torque,
                             float             kp_yaw,
                             WheelTorques     *out);

#endif /* TORQUE_VECTORING_H */
