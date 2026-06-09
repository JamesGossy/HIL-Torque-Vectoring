#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include "../../shared/tv_interface.h"
#include "../../shared/parameters_config.h"

/*
 * torque_vectoring.h
 *
 * The ECU side of the HIL system. In a real car this would run on an embedded
 * controller; here it is compiled into the same binary but only ever sees a
 * SensorData struct, never the full vehicle state. That is the HIL boundary.
 *
 * The algorithm decides how to split the driver's total torque demand across
 * the four wheels to help the car follow the intended path. It is a model-based
 * yaw-moment controller: feedforward from the cornering demand, plus PID
 * feedback on yaw-rate error, with a rear-biased torque split.
 *
 * Runtime tuning: press [ or ] to change KP_YAW by 5, or t to toggle TV on/off.
 * All gains live in shared/parameters_config.h.
 */

/*
 * Compute wheel torques from sensor data.
 *   sensors       what the ECU can sense (yaw rate, speed, steering, wheel speeds)
 *   driver_torque total torque demanded by the driver, Nm
 *   kp_yaw        proportional gain (changed at runtime by the user)
 *   out           filled with the four wheel torque demands
 */
void torque_vectoring_update(
    const SensorData *sensors, float driver_torque, float kp_yaw, WheelTorques *out);

/*
 * Reset the controller's internal PID state. Normal operation never needs this
 * (the state self-heals), but call it to start a clean run or to isolate test
 * cases from each other.
 */
void torque_vectoring_reset(void);

#endif /* TORQUE_VECTORING_H */
