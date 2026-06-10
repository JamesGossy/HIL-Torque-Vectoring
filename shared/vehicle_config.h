#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <math.h>

/* M25 vehicle parameters: measurable and derived physical constants used by both
 * the HIL sim and the ECU. Controller gains live in shared/tunables.c. */

/* ---- geometry ---- */
#define WHEELBASE_M         1.55f
#define TRACK_WIDTH_M       1.30f
#define TRACK_WIDTH_FRONT_M TRACK_WIDTH_M
#define TRACK_WIDTH_REAR_M  TRACK_WIDTH_M
#define WHEEL_RADIUS_M      0.254f
#define GEAR_RATIO          15.47f
#define CG_TO_FRONT_M       0.77f
#define CG_TO_REAR_M        0.78f
#define CG_HEIGHT_M         0.28f

/* ---- mass and inertia ---- */
#define MASS_KG          260.0f
#define YAW_INERTIA_KGM2 140.0f

#define LOAD_TRANSFER_TAU_S 0.08f /* first-order load-transfer lag, s */

/* ---- performance limits ---- */
#define MAX_SPEED_MS    32.0f
#define TARGET_SPEED_MS MAX_SPEED_MS

#define INNER_STEERING_RATIO 0.255625f
#define OUTER_STEERING_RATIO 0.20375f
#define ACK_NOMINAL (0.5f * (INNER_STEERING_RATIO + OUTER_STEERING_RATIO)) /* mid of inner/outer */

/* ---- motors ---- */
/* Motor-shaft Nm. Gear ratio is applied in the vehicle model only, do not pre-multiply elsewhere. */
#define MAX_MOTOR_TORQUE_NM 29.4f
#define MIN_MOTOR_TORQUE_NM -29.4f
#define N_MOTORS            4
#define DRIVER_TORQUE_NM    (N_MOTORS * MAX_MOTOR_TORQUE_NM)
#define DRIVER_BRAKE_NM     (N_MOTORS * MIN_MOTOR_TORQUE_NM)

#define CONTROL_DT_S 0.01f /* 100 Hz */

/* Array capacities. Actual scan depths are runtime tunables clamped to these. */
#define SPEED_PLAN_STEPS_CAP    64
#define NEAREST_SEARCH_BACK_CAP 8
#define NEAREST_SEARCH_FWD_CAP  48

/* ---- tyres and aero ---- */
/* Pacejka lateral tyre model: Fy = D*Fz*sin(C*atan(B*a - E*(B*a - atan(B*a)))) */
#define TYRE_B 12.33675f
#define TYRE_C -1.4203069f
#define TYRE_D 1.43284504f
#define TYRE_E 0.422900f

#define MU_TYRE 1.1f /* peak friction for the yaw-rate limiter */
#define MU_GRIP 1.4f /* friction-circle mu */

#define CLA         5.1f
#define CDA         1.8f
#define AERO_AREA   1.0f
#define AIR_DENSITY 1.29f

#define AERO_GRIP_COEF (MU_GRIP * 0.5f * AIR_DENSITY * CLA * AERO_AREA / MASS_KG) /* lateral grip per v^2 from downforce */

/* Speed-dependent lateral grip in m/s^2, downforce adds a v^2 bonus on top of the flat base. */
static inline float lateral_grip_accel(float base, float v)
{
    return base + AERO_GRIP_COEF * v * v;
}

/* Closed-form apex speed: largest v with v^2*kappa <= base + AERO_GRIP_COEF*v^2. */
static inline float apex_speed(float base, float kappa, float v_cap)
{
    if (kappa <= 1e-4f) return v_cap;
    float denom = kappa - AERO_GRIP_COEF;
    if (denom <= 1e-4f) return v_cap; /* downforce-dominated corner */
    float v = sqrtf(base / denom);
    return (v < v_cap) ? v : v_cap;
}

#endif /* VEHICLE_CONFIG_H */
