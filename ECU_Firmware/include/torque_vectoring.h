#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include "../../shared/tv_interface.h"
#include "../../shared/vehicle_config.h"
#include "../../shared/tunables.h"

/*
 * ECU side of the HIL system. It only ever sees a SensorData struct, never the
 * full vehicle state, which is the HIL boundary. It splits the driver's torque
 * demand across the four wheels: feedforward plus PID on yaw-rate error.
 */

/* ---- public api ---- */

// Compute the four wheel torques from sensor data and the driver torque demand.
void torque_vectoring_update(
    const SensorData *sensors, float driver_torque, float kp_yaw, WheelTorques *out);

// Reset the controller's internal PID state to start a clean run or isolate tests.
void torque_vectoring_reset(void);

#endif /* TORQUE_VECTORING_H */
