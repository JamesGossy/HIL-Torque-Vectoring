#include "../include/vehicle_model.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/*
 * vehicle_model.c  —  M25 2-DOF dynamic bicycle model
 *
 * Improvements over the previous linear model
 * --------------------------------------------
 *
 * 1. Pacejka Magic Formula lateral tyres
 *    The old model used Fy = Ca * α clipped to ±μ*Fz.  The linear region was
 *    too stiff (the car never truly lost grip) and the hard clip caused a
 *    discontinuous jump in tyre force at the limit, making the yaw response
 *    unrealistically abrupt.
 *
 *    Pacejka gives a smooth, load-sensitive curve: force builds quickly at small
 *    slip, peaks, then gently trails off.  The key difference for the TV demo is
 *    that the FRONT axle saturates before the rear (because lf ≈ lr but the
 *    front has slightly less static load), producing visible understeer that TV
 *    can correct.  The smooth saturation also makes the correction continuous
 *    rather than bang-bang.
 *
 * 2. Aerodynamic downforce + quadratic drag
 *    Downforce = 0.5 * ρ * A * CLA * vx² adds to the static axle normal loads,
 *    so available lateral grip increases with speed.  The M25 CLA = 5.1 gives
 *    ~645 N extra load at 14 m/s, raising peak lateral g by ~25 %.
 *    Drag = 0.5 * ρ * A * CDA * vx² replaces the old linear coefficient and
 *    is the physically correct form.
 *
 * 3. M25 parameters
 *    mass=260 kg, Izz=140 kg·m², lf=0.77 m, lr=0.78 m (nearly neutral balance),
 *    track=1.30 m, wheelRadius=0.254 m.  The shorter wheelbase (1.55 m vs the
 *    old 2.4 m) makes the car respond faster to steering — correct for FSG.
 *
 * Structure (unchanged from previous version)
 * --------------------------------------------
 *   1. Drive / drag
 *   2. TV yaw moment from left/right torque differential
 *   3. Normal loads (static + aero downforce + longitudinal transfer)
 *   4. Axle slip angles
 *   5. Lateral tyre forces  ← now Pacejka
 *   6. Equations of motion
 *   7. Integration
 *   8. Position and heading integration
 *   9. Body slip angle
 */


/* ------------------------------------------------------------------ */
/* Pacejka lateral tyre force                                           */
/* ------------------------------------------------------------------ */

/*
 * Returns the lateral force for one axle.
 *
 * alpha — slip angle in our sign convention:
 *           positive = tyre pointing left of travel → force to the left (positive Fy)
 * Fz    — total normal load on the axle, N
 *
 * Formula:  Fy = D * Fz * sin( C * atan( B*κ − E*(B*κ − atan(B*κ)) ) )
 * where κ = −alpha maps our convention to the reference formula's convention
 * (the reference uses κ < 0 for a left-cornering front tyre).
 * With C < 0 this gives Fy > 0 (leftward) for positive alpha. ✓
 */
static float pacejka_lat(float alpha, float Fz)
{
    float k    = -alpha;  /* sign convention: see comment above */
    float Bk   = TYRE_B * k;
    float shape = sinf(TYRE_C * atanf(Bk - TYRE_E * (Bk - atanf(Bk))));
    return TYRE_D * Fz * shape;
}


/* ------------------------------------------------------------------ */
/* Init                                                                 */
/* ------------------------------------------------------------------ */

void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading)
{
    s->x          = start_x;
    s->y          = start_y;
    s->heading    = start_heading;
    s->velocity   = 0.0f;
    s->vy         = 0.0f;
    s->yaw_rate   = 0.0f;
    s->slip_angle = 0.0f;
    s->steering   = 0.0f;
    s->ax         = 0.0f;
    s->ay         = 0.0f;
}


/* ------------------------------------------------------------------ */
/* Physics tick                                                         */
/* ------------------------------------------------------------------ */

void vehicle_model_update(VehicleState *s, const WheelTorques *t, float dt)
{
    float vx = s->velocity;
    float g  = 9.81f;

    /* ------------------------------------------------------------------ */
    /* 1. Aerodynamic forces                                               */
    /* ------------------------------------------------------------------ */

    float q          = 0.5f * AIR_DENSITY * AERO_AREA * vx * vx;  /* dynamic pressure × area */
    float F_downforce = CLA * q;   /* extra normal force (positive = down) */
    float F_drag      = CDA * q;   /* drag force opposing motion           */

    /* ------------------------------------------------------------------ */
    /* 2. Longitudinal drive force                                          */
    /* ------------------------------------------------------------------ */

    float total_torque = t->fl + t->fr + t->rl + t->rr;
    float drive_force  = total_torque / WHEEL_RADIUS_M;
    float net_force    = drive_force - F_drag;
    s->ax              = net_force / MASS_KG;   /* stored for G-G display */

    /* ------------------------------------------------------------------ */
    /* 3. TV yaw moment from left/right torque differential                */
    /* ------------------------------------------------------------------ */

    float torque_diff_front = t->fr - t->fl;
    float torque_diff_rear  = t->rr - t->rl;
    float yaw_moment_tv     = (torque_diff_front + torque_diff_rear)
                              * (TRACK_WIDTH_M * 0.5f) / WHEEL_RADIUS_M;

    /* ------------------------------------------------------------------ */
    /* 4. Normal (vertical) loads                                          */
    /*                                                                     */
    /* Static load split by CG position.                                  */
    /* Aero downforce split 50/50 front and rear.                         */
    /* Longitudinal transfer: positive ax (acceleration) loads the rear.  */
    /* ------------------------------------------------------------------ */

    float Fz_front = MASS_KG * g * (CG_TO_REAR_M  / WHEELBASE_M) + F_downforce * 0.5f;
    float Fz_rear  = MASS_KG * g * (CG_TO_FRONT_M / WHEELBASE_M) + F_downforce * 0.5f;

    float dFz_long = s->ax * MASS_KG * CG_HEIGHT_M / WHEELBASE_M;
    Fz_front -= dFz_long;
    Fz_rear  += dFz_long;

    if (Fz_front < 50.0f) Fz_front = 50.0f;
    if (Fz_rear  < 50.0f) Fz_rear  = 50.0f;

    /* ------------------------------------------------------------------ */
    /* 5. Axle slip angles                                                 */
    /* ------------------------------------------------------------------ */

    float alpha_f = 0.0f;
    float alpha_r = 0.0f;

    if (vx > 0.5f) {
        alpha_f = s->steering
                  - atanf((s->vy + CG_TO_FRONT_M * s->yaw_rate) / vx);
        alpha_r = -atanf((s->vy - CG_TO_REAR_M  * s->yaw_rate) / vx);
    }

    /* ------------------------------------------------------------------ */
    /* 6. Lateral tyre forces  (Pacejka Magic Formula)                    */
    /* ------------------------------------------------------------------ */

    float Fy_f = pacejka_lat(alpha_f, Fz_front);
    float Fy_r = pacejka_lat(alpha_r, Fz_rear);

    /* ------------------------------------------------------------------ */
    /* 7. Equations of motion                                              */
    /*                                                                     */
    /*   M*(dvy/dt) = Fy_f + Fy_r − M*vx*r   (lateral, with Coriolis)   */
    /*   Iz*(dr/dt) = lf*Fy_f − lr*Fy_r + Mtv                           */
    /* ------------------------------------------------------------------ */

    float vy_dot = (Fy_f + Fy_r) / MASS_KG - vx * s->yaw_rate;
    float r_dot  = (CG_TO_FRONT_M * Fy_f
                    - CG_TO_REAR_M  * Fy_r
                    + yaw_moment_tv)
                   / YAW_INERTIA_KGM2;

    s->ay = vy_dot + vx * s->yaw_rate;   /* lateral accel for G-G display */

    /* ------------------------------------------------------------------ */
    /* 8. Integrate states                                                 */
    /* ------------------------------------------------------------------ */

    s->vy       += vy_dot * dt;
    s->yaw_rate += r_dot  * dt;
    s->velocity += s->ax  * dt;

    /* Damp lateral motion at very low speed to avoid division-by-vx blowup */
    if (vx < 0.5f) {
        s->vy       *= 0.85f;
        s->yaw_rate *= 0.90f;
    }

    /* Yaw-rate stability limiter: r cannot exceed peak_mu * g / vx
     * (physical limit set by tyres regardless of applied yaw moment) */
    if (vx > 1.0f) {
        float r_max = (MU_TYRE * g) / vx;
        if (s->yaw_rate >  r_max) s->yaw_rate =  r_max;
        if (s->yaw_rate < -r_max) s->yaw_rate = -r_max;
    }

    if (s->velocity < 0.0f)        s->velocity = 0.0f;
    if (s->velocity > MAX_SPEED_MS) s->velocity = MAX_SPEED_MS;

    /* ------------------------------------------------------------------ */
    /* 9. Integrate heading and position                                   */
    /* ------------------------------------------------------------------ */

    s->heading += s->yaw_rate * dt;

    float cos_h = cosf(s->heading);
    float sin_h = sinf(s->heading);
    s->x += (s->velocity * cos_h - s->vy * sin_h) * dt;
    s->y += (s->velocity * sin_h + s->vy * cos_h) * dt;

    /* ------------------------------------------------------------------ */
    /* 10. Body slip angle                                                 */
    /* ------------------------------------------------------------------ */

    s->slip_angle = (s->velocity > 0.5f) ? atanf(s->vy / s->velocity) : 0.0f;

    while (s->heading >  PI) s->heading -= 2.0f * PI;
    while (s->heading < -PI) s->heading += 2.0f * PI;
}
