#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <math.h> /* sqrtf, for the grip helpers below */

/* shared/vehicle_config.h - M25 vehicle parameters: the measurable/derived
 * physical constants used by both the HIL sim and the ECU. Controller gains live
 * in shared/tunables.c. */

/* ---- M25 geometry ---- */
#define WHEELBASE_M         1.55f /* lf + lr, metres              */
#define TRACK_WIDTH_M       1.30f /* front = rear track width     */
#define TRACK_WIDTH_FRONT_M TRACK_WIDTH_M
#define TRACK_WIDTH_REAR_M  TRACK_WIDTH_M
#define WHEEL_RADIUS_M      0.254f /* nominal tyre rolling radius  */
#define GEAR_RATIO          15.47f /* motor-to-wheel gear ratio    */
#define CG_TO_FRONT_M       0.77f  /* CG -> front axle (lf)        */
#define CG_TO_REAR_M        0.78f  /* CG -> rear axle  (lr)        */
#define CG_HEIGHT_M         0.28f  /* CG height, metres            */

/* ---- M25 mass / inertia ---- */
#define MASS_KG          260.0f /* total vehicle mass, kg       */
#define YAW_INERTIA_KGM2 140.0f /* yaw moment of inertia, kg m^2 */

#define LOAD_TRANSFER_TAU_S 0.08f /* first-order load-transfer lag, s (~roll response) */

/* ---- Speed / steering geometry ---- */
#define MAX_SPEED_MS    32.0f        /* top-speed limiter (enforced in vehicle_model) */
#define TARGET_SPEED_MS MAX_SPEED_MS /* straight-line cruise target = top speed       */

#define INNER_STEERING_RATIO 0.255625f /* steering reference -> inner wheel angle */
#define OUTER_STEERING_RATIO 0.20375f  /* steering reference -> outer wheel angle */
/* nominal reference->wheel scale, derived as the mid of the inner/outer ratios */
#define ACK_NOMINAL (0.5f * (INNER_STEERING_RATIO + OUTER_STEERING_RATIO))

/* ---- Powertrain torque limits (motor-shaft Nm; gear ratio applied in the
 * vehicle model only, do not pre-multiply elsewhere) ---- */
#define MAX_MOTOR_TORQUE_NM 29.4f                            /* per-motor peak drive torque */
#define MIN_MOTOR_TORQUE_NM -29.4f                           /* per-motor regen limit       */
#define N_MOTORS            4                                /* one per wheel               */
#define DRIVER_TORQUE_NM    (N_MOTORS * MAX_MOTOR_TORQUE_NM) /* max total throttle, Nm      */
#define DRIVER_BRAKE_NM     (N_MOTORS * MIN_MOTOR_TORQUE_NM) /* max total regen braking, Nm */

#define CONTROL_DT_S 0.01f /* control period, s (driver + ECU run at 100 Hz) */

/* ---- Driver array CAPACITIES (compile-time; size fixed arrays). The actual scan
 * depths are runtime tunables (g_SPEED_PLAN_STEPS, g_NEAREST_SEARCH_*) clamped to
 * these. ---- */
#define SPEED_PLAN_STEPS_CAP    64 /* max speed-planner scan depth      */
#define NEAREST_SEARCH_BACK_CAP 8  /* max segments searchable behind    */
#define NEAREST_SEARCH_FWD_CAP  48 /* max segments searchable ahead     */

/* ---- Pacejka lateral tyre model (M25 fit): Fy = D*Fz*sin(C*atan(B*a - E*(B*a - atan(B*a)))) ----
 */
#define TYRE_B 12.33675f   /* stiffness factor (slope at zero slip)  */
#define TYRE_C -1.4203069f /* shape factor (peak width; sign conv.)  */
#define TYRE_D 1.43284504f /* peak factor (peak Fy/Fz ~ peak mu)     */
#define TYRE_E 0.422900f   /* curvature factor (slip angle at peak)  */

#define MU_TYRE 1.1f /* effective peak friction for the yaw-rate limiter       */
#define MU_GRIP 1.4f /* friction-circle mu: sqrt(Fx^2+Fy^2) <= MU_GRIP*Fz      */

/* ---- Aerodynamics (M25) ---- */
#define CLA         5.1f  /* lift coefficient (downforce positive) */
#define CDA         1.8f  /* drag coefficient                      */
#define AERO_AREA   1.0f  /* reference area, m^2                   */
#define AIR_DENSITY 1.29f /* air density, kg/m^3                   */

/* extra lateral grip (m/s^2) per (m/s)^2 of speed, from downforce */
#define AERO_GRIP_COEF (MU_GRIP * 0.5f * AIR_DENSITY * CLA * AERO_AREA / MASS_KG)

/* Speed-dependent lateral grip, m/s^2, for a low-speed base budget (downforce
 * adds an additive v^2 bonus on top of the flat budget). */
static inline float lateral_grip_accel(float base, float v)
{
    return base + AERO_GRIP_COEF * v * v;
}

/* Closed-form apex speed: largest v with v^2*kappa <= base + AERO_GRIP_COEF*v^2.
 * Returns v_cap when the corner is open/downforce-dominated. */
static inline float apex_speed(float base, float kappa, float v_cap)
{
    if (kappa <= 1e-4f) return v_cap;
    float denom = kappa - AERO_GRIP_COEF;
    if (denom <= 1e-4f) return v_cap; /* downforce-dominated corner */
    float v = sqrtf(base / denom);
    return (v < v_cap) ? v : v_cap;
}

#endif /* VEHICLE_CONFIG_H */
