#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/* =========================================================================
 * vehicle_config.h  —  M25 vehicle parameters
 *
 * Edit this file to tune the model. All defines here are consumed by
 * vehicle_model.h / vehicle_model.c — do not change those files directly.
 * ========================================================================= */

/* ---- M25 geometry ---- */
#define WHEELBASE_M      1.55f   /* lf + lr, metres                        */
#define CG_TO_FRONT_M    0.77f   /* CG → front axle (lf)                  */
#define CG_TO_REAR_M     0.78f   /* CG → rear axle  (lr)                  */
#define TRACK_WIDTH_M    1.30f   /* front and rear track width             */
#define CG_HEIGHT_M      0.28f   /* CG height, metres                      */

/* ---- M25 mass / inertia ---- */
#define MASS_KG          260.0f
#define YAW_INERTIA_KGM2 140.0f

/* ---- Wheel / drivetrain ---- */
#define WHEEL_RADIUS_M   0.254f
#define MAX_SPEED_MS      30.0f

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
