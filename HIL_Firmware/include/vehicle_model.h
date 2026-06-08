#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "../../shared/tv_interface.h"
#include "../../shared/vehicle_config.h"

/*
 * vehicle_model.h  --  M25 dynamic vehicle model (4-corner)
 *
 * A 3-DOF planar model (vx, vy, yaw) evaluated per wheel: per-wheel Pacejka
 * lateral tyres, quadratic aero downforce and drag, longitudinal and lateral
 * load transfer, Ackermann steering, and per-wheel drive force from motor
 * torque (which is what makes torque vectoring possible).
 *
 * Physical constants (mass, geometry, tyre coefficients) live in
 * shared/vehicle_config.h.
 */


typedef struct {
    float x;               /* World X position, metres (east +)            */
    float y;               /* World Y position, metres (north +)           */
    float heading;         /* Yaw angle, rad.  0 = east, CCW +             */
    float velocity;        /* Longitudinal speed vx, m/s                   */
    float vy;              /* Lateral velocity at CG, m/s (left +)         */
    float yaw_rate;        /* Yaw rate, rad/s (CCW +, i.e. turning left +) */
    float slip_angle;      /* Body slip angle β = atan2(vy, vx), rad       */

    /* Steering: set steering_input to the common front wheel angle (rad).
     * The model computes FL/FR individually using Ackermann ratios. */
    float steering;        /* Signed front-axle reference angle, rad       */
    float steer_fl;        /* Computed FL wheel angle (Ackermann), rad     */
    float steer_fr;        /* Computed FR wheel angle (Ackermann), rad     */

    float ax;              /* Longitudinal acceleration, m/s² (G-G display)*/
    float ay;              /* Lateral acceleration, m/s² (G-G display)     */

    /* Roll/pitch-lagged accelerations driving load transfer (first-order
     * filtered ax/ay).  Kept as state so the transfer terms do not form an
     * algebraic feedback loop with the same-tick acceleration. */
    float ax_filt;         /* Lagged longitudinal accel for load transfer  */
    float ay_filt;         /* Lagged lateral accel for load transfer       */

    /* Per-wheel outputs (RPM in motor shaft frame) */
    float wheelspeed[4];   /* FL, FR, RL, RR - motor RPM (positive = fwd) */

} VehicleState;


/* All vehicle parameters are defined in shared/vehicle_config.h - edit that file. */

/* Set the vehicle to its starting position and zero all motion. */
void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading);

/*
 * Advance the vehicle physics by one time step.
 *
 * s       - current vehicle state (updated in place)
 * torques - wheel torques from the ECU (Nm each; positive = drive, negative = regen)
 * dt      - time step in seconds
 */
void vehicle_model_update(VehicleState *s, const WheelTorques *torques, float dt);

#endif /* VEHICLE_MODEL_H */
