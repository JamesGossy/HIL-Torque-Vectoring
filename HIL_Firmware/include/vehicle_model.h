#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "../../shared/tv_interface.h"
#include "vehicle_config.h"

/*
 * vehicle_model.h  —  M25 dynamic vehicle model (4-corner)
 *
 * 3-DOF planar model (vx, vy, yaw) evaluated per wheel:
 *   • Pacejka "Magic Formula" lateral tyre forces, run PER WHEEL
 *     (load-sensitive, nonlinear)
 *   • Quadratic aerodynamic downforce + drag
 *   • Longitudinal AND lateral load transfer → individual wheel normal loads
 *   • Ackermann per-wheel steering (inner/outer ratios)
 *   • Per-wheel longitudinal force from motor torque (enables torque vectoring)
 *
 *
 * Equations of motion (summed over the four wheels i)
 * ---------------------------------------------------
 * Each wheel force (Fx_i, Fy_i) is rotated into the body frame by its steer
 * angle δ_i, then summed about the CG at corner position (rx_i, ry_i):
 *
 *   M * (dvy/dt + vx*r) = Σ Fy_body_i
 *   Iz * dr/dt          = Σ ( rx_i*Fy_body_i − ry_i*Fx_body_i )
 *
 * The Σ(rx*Fy) terms give the conventional cornering yaw moment; the
 * Σ(ry*Fx) terms give the torque-vectoring moment from the left/right
 * drive-force differential.
 *
 *
 * Tyre model  —  Pacejka Magic Formula (lateral, per wheel)
 * ----------------------------------------------------------
 * Fy = D * Fz * sin( C * atan( B*α − E*(B*α − atan(B*α)) ) )
 *
 *   B  — stiffness factor  (slope near zero slip)
 *   C  — shape factor      (controls peak and tail)
 *   D  — peak factor       (peak Fy / Fz)
 *   E  — curvature factor  (shifts peak slip angle)
 *   α  — slip angle (rad) from THAT wheel's own velocity vector
 *   Fz — normal load on THAT wheel (N), incl. lateral/longitudinal transfer
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
    float wheelspeed[4];   /* FL, FR, RL, RR — motor RPM (positive = fwd) */

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
