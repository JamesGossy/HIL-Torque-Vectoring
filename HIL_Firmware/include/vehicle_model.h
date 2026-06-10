#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "../../shared/tv_interface.h"
#include "../../shared/vehicle_config.h"

/*
 * M25 4-corner dynamic vehicle model. A 3-DOF planar model (vx, vy, yaw) with
 * per-wheel Pacejka tyres, aero, load transfer, Ackermann steering and per-wheel
 * drive force. Physical constants live in shared/vehicle_config.h.
 */


typedef struct {
    /* ---- pose and motion ---- */
    float x;          /* World X position, metres (east +) */
    float y;          /* World Y position, metres (north +) */
    float heading;    /* Yaw angle, rad. 0 = east, CCW + */
    float velocity;   /* Longitudinal speed vx, m/s */
    float vy;         /* Lateral velocity at CG, m/s (left +) */
    float yaw_rate;   /* Yaw rate, rad/s (CCW + is turning left) */
    float slip_angle; /* Body slip angle, rad */

    /* ---- steering ---- */
    float steering; /* Signed front-axle reference angle, rad. Model derives FL/FR via Ackermann */
    float steer_fl; /* Computed FL wheel angle (Ackermann), rad */
    float steer_fr; /* Computed FR wheel angle (Ackermann), rad */

    /* ---- accelerations ---- */
    float ax; /* Longitudinal acceleration, m/s2 (G-G display) */
    float ay; /* Lateral acceleration, m/s2 (G-G display) */

    float ax_filt; /* Lagged longitudinal accel, kept as state so load transfer is not an algebraic loop */
    float ay_filt; /* Lagged lateral accel, same reason */

    float wheelspeed[4]; /* FL, FR, RL, RR motor RPM (positive = fwd) */

} VehicleState;


/* Set the vehicle to its starting position and zero all motion. */
void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading);

/* Advance the vehicle physics by one time step (dt seconds), updating s in place. */
void vehicle_model_update(VehicleState *s, const WheelTorques *torques, float dt);

#endif /* VEHICLE_MODEL_H */
