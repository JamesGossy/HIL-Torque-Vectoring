#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <math.h> /* sqrtf, for the grip helpers below */

/* =========================================================================
 * shared/vehicle_config.h  -  M25 vehicle parameters
 *
 * Single source of truth for all vehicle constants used by both the HIL
 * simulation and the ECU firmware.  Edit this file to tune the model.
 * ========================================================================= */

/* ---- M25 geometry ---- */
#define WHEELBASE_M         1.55f /* lf + lr, metres                    */
#define TRACK_WIDTH_M       1.30f /* front = rear track width           */
#define TRACK_WIDTH_FRONT_M TRACK_WIDTH_M
#define TRACK_WIDTH_REAR_M  TRACK_WIDTH_M
#define WHEEL_RADIUS_M      0.254f /* nominal tyre rolling radius        */
#define GEAR_RATIO          15.47f /* motor-to-wheel gear ratio          */
#define CG_TO_FRONT_M       0.77f  /* CG → front axle (lf)              */
#define CG_TO_REAR_M        0.78f  /* CG → rear axle  (lr)              */
#define CG_HEIGHT_M         0.28f  /* CG height, metres                  */

/* ---- M25 mass / inertia ---- */
#define MASS_KG          260.0f
#define YAW_INERTIA_KGM2 140.0f

/* ---- Load-transfer lag ----
 * Lateral/longitudinal load transfer does not appear instantly: it builds as
 * the chassis rolls/pitches against the suspension.  We model that with a
 * first-order lag (time constant LOAD_TRANSFER_TAU_S) on the acceleration that
 * drives the load-transfer terms.  Without this lag the transfer is computed
 * algebraically from the same-tick acceleration it helps produce, forming an
 * undamped feedback loop that rings at the sim step rate - seen as violently
 * jittery yaw rate and wheel torques.  ~80 ms is a representative roll-response
 * time for a stiff race car. */
#define LOAD_TRANSFER_TAU_S 0.08f

/* ---- Wheel / drivetrain ---- */
#define MAX_SPEED_MS 30.0f

/* ---- Ackermann steering ratios ---- */
/* steer_wheel_angle * ratio = wheel angle (rad)                             */
#define INNER_STEERING_RATIO 0.255625f
#define OUTER_STEERING_RATIO 0.20375f


/* ---- Pacejka lateral tyre model (M25 fitted coefficients) ----
 *
 * Fy = D * Fz * sin( C * atan( B*α − E*(B*α − atan(B*α)) ) )
 *
 *   B  - stiffness factor  (slope at zero slip; higher = sharper build-up)
 *   C  - shape factor      (controls peak width; negative here for sign convention)
 *   D  - peak factor       (peak Fy / Fz; ~= peak friction coefficient)
 *   E  - curvature factor  (shifts the slip angle at peak; 0 = symmetric)
 */
#define TYRE_B 12.33675f
#define TYRE_C -1.4203069f
#define TYRE_D 1.43284504f
#define TYRE_E 0.422900f

/* Effective peak friction for the yaw-rate stability limiter */
#define MU_TYRE 1.1f

/* Peak friction coefficient for the per-wheel combined-slip FRICTION CIRCLE
 * (vehicle_model.c step 6b): sqrt(Fx^2 + Fy^2) <= MU_GRIP * Fz.  Set near the
 * Pacejka lateral peak factor (TYRE_D ~ 1.43) so a pure-cornering tyre is
 * essentially unaffected and only the COMBINED longitudinal+lateral demand is
 * clipped.  This is the model's primary grip limiter now; the r <= mu*g/v yaw
 * clamp is only a numerical safety net behind it. */
#define MU_GRIP 1.4f

/* ---- Aerodynamics (M25) ---- */
#define CLA         5.1f  /* lift coefficient (downforce positive)       */
#define CDA         1.8f  /* drag coefficient                            */
#define AERO_AREA   1.0f  /* reference area, m²                         */
#define AIR_DENSITY 1.29f /* kg/m³                                      */


/* ---- Speed-dependent lateral grip (downforce) ----
 *
 * The tyre's lateral grip is not constant: aero downforce adds normal load that
 * scales with v^2, so the car can corner harder the faster it goes. The line
 * optimiser and the on-car speed planner both need this, or they draw/drive a
 * line tuned for the gripless low-speed car and leave time on the table in every
 * fast corner (the error reaches ~40% of grip near 18 m/s on this car).
 *
 *   a_lat_max(v) = base + AERO_GRIP_COEF * v^2
 *
 * where `base` is the caller's low-speed (zero-downforce) lateral budget
 * (MAX_LATERAL_ACCEL_MS2 / PP_GRIP_ACCEL) and the aero term is the extra grip
 * from downforce. Keeping the downforce as an additive BONUS on top of the
 * existing flat budget means all existing low-speed tuning (notably the hairpin,
 * where there is no downforce) is unchanged; only the fast corners gain grip.
 *
 *   AERO_GRIP_COEF = MU_GRIP * (0.5 * rho * CLA * AERO_AREA) / m
 *                  = extra m/s^2 of lateral grip per (m/s)^2 of speed.
 */
#define AERO_GRIP_COEF (MU_GRIP * 0.5f * AIR_DENSITY * CLA * AERO_AREA / MASS_KG)

/* Speed-dependent lateral grip, m/s^2, for a given low-speed base budget. */
static inline float lateral_grip_accel(float base, float v)
{
    return base + AERO_GRIP_COEF * v * v;
}

/*
 * Closed-form apex speed: the largest v that satisfies v^2*kappa <= a_lat_max(v)
 * with the speed-dependent grip above. Solving
 *     v^2 * kappa = base + AERO_GRIP_COEF * v^2
 *  => v^2 (kappa - AERO_GRIP_COEF) = base
 *  => v = sqrt( base / (kappa - AERO_GRIP_COEF) )
 * No iteration needed. If kappa <= AERO_GRIP_COEF the corner is so open that
 * downforce alone holds any speed (radius > ~56 m here) - return v_cap. */
static inline float apex_speed(float base, float kappa, float v_cap)
{
    if (kappa <= 1e-4f) return v_cap;
    float denom = kappa - AERO_GRIP_COEF;
    if (denom <= 1e-4f) return v_cap; /* downforce-dominated corner */
    float v = sqrtf(base / denom);
    return (v < v_cap) ? v : v_cap;
}

#endif /* VEHICLE_CONFIG_H */
