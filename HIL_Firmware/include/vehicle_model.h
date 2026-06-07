#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "../../shared/tv_interface.h"
#include "vehicle_config.h"

/*
 * vehicle_model.h  —  M25 dynamic bicycle model
 *
 * 2-DOF dynamic bicycle with:
 *   • Pacejka "Magic Formula" lateral tyre forces (load-sensitive, nonlinear)
 *   • Quadratic aerodynamic downforce + drag
 *   • Longitudinal and lateral load transfer
 *
 *
 * Equations of motion
 * -------------------
 *   M  * (dvy/dt + vx*r) = Fy_f + Fy_r            (lateral force balance)
 *   Iz * dr/dt           = lf*Fy_f − lr*Fy_r + Mtv (yaw moment balance)
 *
 *
 * Tyre model  —  Pacejka Magic Formula (lateral)
 * -----------------------------------------------
 * Fy = D * Fz * sin( C * atan( B*α − E*(B*α − atan(B*α)) ) )
 *
 *   B  — stiffness factor  (slope near zero slip)
 *   C  — shape factor      (controls peak and tail)
 *   D  — peak factor       (peak Fy / Fz)
 *   E  — curvature factor  (shifts peak slip angle)
 *   α  — slip angle (rad); positive = tyre pointing left of travel
 *   Fz — normal load on the axle (N)
 *
 * MU_TYRE is the effective peak lateral friction used only by the yaw-rate
 * limiter; it approximates the true Pacejka peak under representative load.
 *
 *
 * Aerodynamics
 * ------------
 * F_downforce = 0.5 * AIR_DENSITY * AERO_AREA * CLA * vx²
 * F_drag      = 0.5 * AIR_DENSITY * AERO_AREA * CDA * vx²
 *
 * Downforce is split 50/50 front/rear and added to the static axle loads,
 * so the tyres gain grip at higher speeds (as on a real aero car).
 */


typedef struct {
    float x;          /* World X position, metres (east +)                */
    float y;          /* World Y position, metres (north +)               */
    float heading;    /* Yaw angle, rad.  0 = east, CCW +                 */
    float velocity;   /* Longitudinal speed vx, m/s                       */
    float vy;         /* Lateral velocity at CG, m/s (right +)            */
    float yaw_rate;   /* dr/dt, rad/s                                     */
    float slip_angle; /* Body slip angle β = atan2(vy, vx), rad           */
    float steering;   /* Front wheel steer angle, rad (set by controller) */
    float ax;         /* Longitudinal acceleration, m/s² (G-G display)    */
    float ay;         /* Lateral acceleration, m/s² (G-G display)         */
} VehicleState;


/* All vehicle parameters are defined in vehicle_config.h — edit that file. */

/* Set the vehicle to its starting position and zero all motion. */
void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading);

/*
 * Advance the vehicle physics by one time step.
 *
 * s       — current vehicle state (updated in place)
 * torques — wheel torques from the ECU (Nm each; positive = drive, negative = regen)
 * dt      — time step in seconds
 */
void vehicle_model_update(VehicleState *s, const WheelTorques *torques, float dt);

#endif /* VEHICLE_MODEL_H */
