#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/* =========================================================================
 * shared/vehicle_config.h  —  M25 vehicle parameters
 *
 * Single source of truth for all vehicle constants used by both the HIL
 * simulation and the ECU firmware.  Edit this file to tune the model.
 * ========================================================================= */

/* ---- M25 geometry ---- */
#define WHEELBASE_M          1.55f   /* lf + lr, metres                    */
#define TRACK_WIDTH_M        1.30f   /* front = rear track width           */
#define TRACK_WIDTH_FRONT_M  TRACK_WIDTH_M
#define TRACK_WIDTH_REAR_M   TRACK_WIDTH_M
#define WHEEL_RADIUS_M       0.254f  /* nominal tyre rolling radius        */
#define GEAR_RATIO           15.47f  /* motor-to-wheel gear ratio          */
#define CG_TO_FRONT_M        0.77f   /* CG → front axle (lf)              */
#define CG_TO_REAR_M         0.78f   /* CG → rear axle  (lr)              */
#define CG_HEIGHT_M          0.28f   /* CG height, metres                  */

/* ---- M25 mass / inertia ---- */
#define MASS_KG          260.0f
#define YAW_INERTIA_KGM2 140.0f

/* ---- Load-transfer lag ----
 * Lateral/longitudinal load transfer does not appear instantly: it builds as
 * the chassis rolls/pitches against the suspension.  We model that with a
 * first-order lag (time constant LOAD_TRANSFER_TAU_S) on the acceleration that
 * drives the load-transfer terms.  Without this lag the transfer is computed
 * algebraically from the same-tick acceleration it helps produce, forming an
 * undamped feedback loop that rings at the sim step rate — seen as violently
 * jittery yaw rate and wheel torques.  ~80 ms is a representative roll-response
 * time for a stiff race car. */
#define LOAD_TRANSFER_TAU_S  0.08f

/* ---- Wheel / drivetrain ---- */
#define MAX_SPEED_MS     30.0f

/* ---- Ackermann steering ratios ---- */
/* steer_wheel_angle * ratio = wheel angle (rad)                             */
#define INNER_STEERING_RATIO  0.255625f
#define OUTER_STEERING_RATIO  0.20375f


/* ---- Pacejka lateral tyre model (M25 fitted coefficients) ----
 *
 * Fy = D * Fz * sin( C * atan( B*α − E*(B*α − atan(B*α)) ) )
 *
 *   B  — stiffness factor  (slope at zero slip; higher = sharper build-up)
 *   C  — shape factor      (controls peak width; negative here for sign convention)
 *   D  — peak factor       (peak Fy / Fz; ~= peak friction coefficient)
 *   E  — curvature factor  (shifts the slip angle at peak; 0 = symmetric)
 */
#define TYRE_B   12.33675f
#define TYRE_C   -1.4203069f
#define TYRE_D    1.43284504f
#define TYRE_E    0.422900f

/* Effective peak friction for the yaw-rate stability limiter */
#define MU_TYRE  1.1f

/* ---- Aerodynamics (M25) ---- */
#define CLA          5.1f    /* lift coefficient (downforce positive)       */
#define CDA          1.8f    /* drag coefficient                            */
#define AERO_AREA    1.0f    /* reference area, m²                         */
#define AIR_DENSITY  1.29f   /* kg/m³                                      */

#endif /* VEHICLE_CONFIG_H */
