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
 *    so available lateral grip increases with speed.
 *    Drag = 0.5 * ρ * A * CDA * vx² is the physically correct quadratic form.
 *
 * 3. M25 parameters
 *    mass=260 kg, Izz=140 kg·m², lf=0.77 m, lr=0.78 m (nearly neutral balance),
 *    track=1.30 m, wheelRadius=0.254 m.
 *
 * 4. Ackermann per-wheel steering geometry
 *    Inner and outer wheel angles are computed separately from the signed
 *    reference steering input using INNER_STEERING_RATIO / OUTER_STEERING_RATIO.
 *    Per-wheel FL/FR steer angles are used throughout the force calculations.
 *
 * 5. Full 4-corner model (the big accuracy gain)
 *    Each wheel is treated individually rather than lumped into front/rear
 *    axles:
 *      • Velocity at each corner = v_CoG + ω × r_corner, so the slip angle is
 *        measured from that tyre's own velocity vector.
 *      • Normal load per wheel includes lateral load transfer (the outer tyres
 *        gain load, the inner tyres shed it), on top of the static split, aero
 *        downforce, and longitudinal transfer.
 *      • Pacejka runs per wheel, so the loaded outer tyres carry most of the
 *        cornering force and the unloaded inner tyres saturate first — the load
 *        sensitivity that gives torque vectoring something to exploit.
 *      • Each wheel's Fx/Fy is rotated into the body frame through its own
 *        steer angle and summed into the force/yaw balance about the CG.
 *    A stillstand guard zeroes slip below 0.1 m/s to avoid atan blow-up at rest.
 *
 * 6. Per-wheel RPM outputs
 *    Wheelspeeds (motor RPM) are computed from each corner's longitudinal
 *    velocity component and fed back into SensorData for the ECU — the ECU can
 *    now sense the real left/right speed split, not a single faked value.
 *
 * Structure
 * ---------
 *   1. Ackermann steering
 *   2. Aerodynamic forces (downforce + drag)
 *   3. Per-wheel normal loads (static + aero + longitudinal + lateral transfer)
 *   4. 4-corner velocities and per-wheel slip angles
 *   5. Lateral tyre forces  (Pacejka, per wheel)
 *   6. Per-wheel longitudinal drive force
 *   7. Equations of motion (per-wheel force/moment summation)
 *   8. Integration
 *   9. Position and heading integration
 *  10. Per-wheel RPM outputs
 *  11. Body slip angle
 */


/* ------------------------------------------------------------------ */
/* Pacejka lateral tyre force                                           */
/* ------------------------------------------------------------------ */

/*
 * Returns the lateral force for one axle.
 *
 * alpha — slip angle:  positive = tyre pointing left of travel → Fy left (+)
 * Fz    — total normal load on the axle, N
 *
 * Formula:  Fy = D * Fz * sin( C * atan( B*κ − E*(B*κ − atan(B*κ)) ) )
 * κ = −alpha maps our sign convention to the reference formula's convention.
 * With C < 0 this gives Fy > 0 (leftward) for positive alpha. ✓
 */
static float pacejka_lat(float alpha, float Fz)
{
    float k     = -alpha;
    float Bk    = TYRE_B * k;
    float shape = sinf(TYRE_C * atanf(Bk - TYRE_E * (Bk - atanf(Bk))));
    return TYRE_D * Fz * shape;
}


/* ------------------------------------------------------------------ */
/* Init                                                                 */
/* ------------------------------------------------------------------ */

void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading)
{
    s->x                    = start_x;
    s->y                    = start_y;
    s->heading              = start_heading;
    s->velocity             = 0.0f;
    s->vy                   = 0.0f;
    s->yaw_rate             = 0.0f;
    s->slip_angle           = 0.0f;
    s->steering             = 0.0f;
    s->steer_fl             = 0.0f;
    s->steer_fr             = 0.0f;
    s->ax                   = 0.0f;
    s->ay                   = 0.0f;
    s->ax_filt              = 0.0f;
    s->ay_filt              = 0.0f;
    for (int i = 0; i < 4; i++)
        s->wheelspeed[i] = 0.0f;
}


/* ------------------------------------------------------------------ */
/* Physics tick                                                         */
/* ------------------------------------------------------------------ */

void vehicle_model_update(VehicleState *s, const WheelTorques *t, float dt)
{
    float vx = s->velocity;
    float vy = s->vy;
    float r  = s->yaw_rate;
    float g  = 9.81f;

    /* ------------------------------------------------------------------ */
    /* 1. Ackermann per-wheel steering angles                               */
    /* ------------------------------------------------------------------ */

    if (s->steering > 0.0f) {
        s->steer_fl = INNER_STEERING_RATIO * s->steering;
        s->steer_fr = OUTER_STEERING_RATIO * s->steering;
    } else {
        s->steer_fl = OUTER_STEERING_RATIO * s->steering;
        s->steer_fr = INNER_STEERING_RATIO * s->steering;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Aerodynamic forces                                                */
    /* ------------------------------------------------------------------ */

    float q           = 0.5f * AIR_DENSITY * AERO_AREA * vx * vx;
    float F_downforce = CLA * q;
    float F_drag      = CDA * q;

    /* ------------------------------------------------------------------ */
    /* 3. Normal loads — per wheel                                          */
    /*    static split + aero (50/50) + longitudinal + lateral transfer    */
    /*                                                                     */
    /* Longitudinal transfer (ax > 0 → loads rear) is shared across the    */
    /* two wheels of an axle.  Lateral transfer (ay > 0 → loads the right  */
    /* wheels, since +ay points left) is split front/rear by the static    */
    /* axle-load fraction — a good approximation without an explicit roll  */
    /* model.  Per-wheel Fz lets the Pacejka tyres saturate individually,  */
    /* so the heavily-loaded outer tyres carry most of the cornering force  */
    /* and the lightly-loaded inner tyres give up earlier — exactly the    */
    /* load sensitivity that makes torque vectoring worthwhile.            */
    /* ------------------------------------------------------------------ */

    float Fz_front_axle = MASS_KG * g * (CG_TO_REAR_M  / WHEELBASE_M) + F_downforce * 0.5f;
    float Fz_rear_axle  = MASS_KG * g * (CG_TO_FRONT_M / WHEELBASE_M) + F_downforce * 0.5f;

    /* Load transfer is driven by the roll/pitch-lagged accelerations (see
     * shared/vehicle_config.h).  Using the lagged values — not the same-tick ax/ay
     * that the transfer itself helps create — is what keeps the model from
     * ringing tick-to-tick. */
    float dFz_long = s->ax_filt * MASS_KG * CG_HEIGHT_M / WHEELBASE_M;
    Fz_front_axle -= dFz_long;
    Fz_rear_axle  += dFz_long;

    if (Fz_front_axle < 50.0f) Fz_front_axle = 50.0f;
    if (Fz_rear_axle  < 50.0f) Fz_rear_axle  = 50.0f;

    /* Lateral load transfer per axle: ΔFz = m_axle * ay * h / track.        */
    /* m_axle is that axle's share of sprung mass (by the static CG split);   */
    /* using the static split here avoids feeding aero/longitudinal transfer  */
    /* back into the lateral term. */
    float m_front = MASS_KG * (CG_TO_REAR_M  / WHEELBASE_M);
    float m_rear  = MASS_KG * (CG_TO_FRONT_M / WHEELBASE_M);
    float dFz_lat_front = m_front * s->ay_filt * CG_HEIGHT_M / TRACK_WIDTH_FRONT_M;
    float dFz_lat_rear  = m_rear  * s->ay_filt * CG_HEIGHT_M / TRACK_WIDTH_REAR_M;

    /* +ay loads the RIGHT wheels (FR, RR); left wheels (FL, RL) unload */
    float Fz_fl = 0.5f * Fz_front_axle - dFz_lat_front;
    float Fz_fr = 0.5f * Fz_front_axle + dFz_lat_front;
    float Fz_rl = 0.5f * Fz_rear_axle  - dFz_lat_rear;
    float Fz_rr = 0.5f * Fz_rear_axle  + dFz_lat_rear;

    if (Fz_fl < 25.0f) Fz_fl = 25.0f;
    if (Fz_fr < 25.0f) Fz_fr = 25.0f;
    if (Fz_rl < 25.0f) Fz_rl = 25.0f;
    if (Fz_rr < 25.0f) Fz_rr = 25.0f;

    /* ------------------------------------------------------------------ */
    /* 4. 4-corner velocities and per-wheel slip angles                     */
    /*                                                                     */
    /* v_corner = v_CoG + ω × r_corner  (2-D: ω = r k̂)                   */
    /*   v_corner_x = vx − r * r_corner_y                                 */
    /*   v_corner_y = vy + r * r_corner_x                                 */
    /*                                                                     */
    /* Each tyre's slip angle is measured from ITS OWN velocity vector, so  */
    /* the toe-out from Ackermann and the fore/aft yaw-induced lateral      */
    /* velocity are captured per corner rather than lumped at the axle      */
    /* mid-point.                                                          */
    /* ------------------------------------------------------------------ */

    float half_sf = TRACK_WIDTH_FRONT_M * 0.5f;
    float half_sr = TRACK_WIDTH_REAR_M  * 0.5f;

    /* Corner positions in vehicle frame (x = forward, y = left) */
    float rx_fl =  CG_TO_FRONT_M,  ry_fl =  half_sf;
    float rx_fr =  CG_TO_FRONT_M,  ry_fr = -half_sf;
    float rx_rl = -CG_TO_REAR_M,   ry_rl =  half_sr;
    float rx_rr = -CG_TO_REAR_M,   ry_rr = -half_sr;

    float vx_fl = vx - r * ry_fl,  vy_fl = vy + r * rx_fl;
    float vx_fr = vx - r * ry_fr,  vy_fr = vy + r * rx_fr;
    float vx_rl = vx - r * ry_rl,  vy_rl = vy + r * rx_rl;
    float vx_rr = vx - r * ry_rr,  vy_rr = vy + r * rx_rr;

    float eps = 1e-5f;

    /* Stillstand guard: zero slip below 0.1 m/s */
    int stillstand = (vx * vx + vy * vy < 0.01f) && (fabsf(r) < 0.001f);

    float alpha_fl = 0.0f, alpha_fr = 0.0f;
    float alpha_rl = 0.0f, alpha_rr = 0.0f;

    if (!stillstand && vx > 0.5f) {
        /* Per-wheel slip = wheel steer angle − velocity-vector angle */
        alpha_fl = s->steer_fl - atanf(vy_fl / ((fabsf(vx_fl) > eps) ? vx_fl : eps));
        alpha_fr = s->steer_fr - atanf(vy_fr / ((fabsf(vx_fr) > eps) ? vx_fr : eps));
        alpha_rl =             - atanf(vy_rl / ((fabsf(vx_rl) > eps) ? vx_rl : eps));
        alpha_rr =             - atanf(vy_rr / ((fabsf(vx_rr) > eps) ? vx_rr : eps));
    }

    /* ------------------------------------------------------------------ */
    /* 5. Lateral tyre forces  (Pacejka, per wheel)                        */
    /* ------------------------------------------------------------------ */

    float Fy_fl = pacejka_lat(alpha_fl, Fz_fl);
    float Fy_fr = pacejka_lat(alpha_fr, Fz_fr);
    float Fy_rl = pacejka_lat(alpha_rl, Fz_rl);
    float Fy_rr = pacejka_lat(alpha_rr, Fz_rr);

    /* ------------------------------------------------------------------ */
    /* 6. Per-wheel longitudinal drive force                                */
    /*                                                                     */
    /* Gate: suppress force when the wheel has a small command AND the car  */
    /* is barely moving (avoids creep artefacts at standstill).            */
    /* ------------------------------------------------------------------ */

    float rpm2ms = WHEEL_RADIUS_M * 2.0f * PI / (GEAR_RATIO * 60.0f);

    float Fx_fl = GEAR_RATIO * t->fl / WHEEL_RADIUS_M;
    float Fx_fr = GEAR_RATIO * t->fr / WHEEL_RADIUS_M;
    float Fx_rl = GEAR_RATIO * t->rl / WHEEL_RADIUS_M;
    float Fx_rr = GEAR_RATIO * t->rr / WHEEL_RADIUS_M;

    Fx_fl *= ((t->fl > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;
    Fx_fr *= ((t->fr > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;
    Fx_rl *= ((t->rl > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;
    Fx_rr *= ((t->rr > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;

    /* ------------------------------------------------------------------ */
    /* 6b. Combined-slip friction circle (per wheel)                        */
    /*                                                                     */
    /* A tyre has ONE friction budget shared between longitudinal (drive/  */
    /* brake) and lateral (cornering) force: sqrt(Fx^2 + Fy^2) <= mu*Fz.    */
    /* The old model computed Fx straight from motor torque with NO limit,  */
    /* so a wheel could put down arbitrary drive force while already at its  */
    /* lateral grip limit — the car could accelerate hard mid-corner with   */
    /* no penalty (measured: up to 2.4x the tyres' real longitudinal grip). */
    /* That removed the very lateral-vs-longitudinal trade-off that torque  */
    /* vectoring exists to manage.                                          */
    /*                                                                     */
    /* Here each wheel's (Fx, Fy) is projected back onto its own friction   */
    /* circle of radius mu*Fz_i (Fz_i already carries load transfer, so the */
    /* loaded outer tyres keep a bigger budget than the unloaded inner ones */
    /* — the load sensitivity TV exploits).  BOTH components are scaled by   */
    /* the same factor so the resultant lands on the circle: powering up     */
    /* mid-corner now bleeds lateral grip, which is the physical effect the  */
    /* controller's traction-circle throttle cut is meant to pre-empt.       */
    /*                                                                     */
    /* MU_GRIP is set near the Pacejka lateral peak (~D), so a pure-cornering */
    /* tyre (Fx≈0) is essentially unaffected — we only clip the COMBINED      */
    /* demand, we do not weaken the lateral model the car already produces.   */
    /* ------------------------------------------------------------------ */
    {
        float Fmax_fl = MU_GRIP * Fz_fl;
        float Fmax_fr = MU_GRIP * Fz_fr;
        float Fmax_rl = MU_GRIP * Fz_rl;
        float Fmax_rr = MU_GRIP * Fz_rr;

        float c_fl = sqrtf(Fx_fl*Fx_fl + Fy_fl*Fy_fl);
        float c_fr = sqrtf(Fx_fr*Fx_fr + Fy_fr*Fy_fr);
        float c_rl = sqrtf(Fx_rl*Fx_rl + Fy_rl*Fy_rl);
        float c_rr = sqrtf(Fx_rr*Fx_rr + Fy_rr*Fy_rr);

        if (c_fl > Fmax_fl) { float k = Fmax_fl/c_fl; Fx_fl *= k; Fy_fl *= k; }
        if (c_fr > Fmax_fr) { float k = Fmax_fr/c_fr; Fx_fr *= k; Fy_fr *= k; }
        if (c_rl > Fmax_rl) { float k = Fmax_rl/c_rl; Fx_rl *= k; Fy_rl *= k; }
        if (c_rr > Fmax_rr) { float k = Fmax_rr/c_rr; Fx_rr *= k; Fy_rr *= k; }
    }

    float cos_fl = cosf(s->steer_fl), sin_fl = sinf(s->steer_fl);
    float cos_fr = cosf(s->steer_fr), sin_fr = sinf(s->steer_fr);

    /* ------------------------------------------------------------------ */
    /* 7. Equations of motion                                               */
    /*                                                                     */
    /* Per-wheel front tyre forces resolve into the vehicle frame through    */
    /* that wheel's own steer angle:                                         */
    /*   Fx_body =  cos(δ)*Fx − sin(δ)*Fy                                   */
    /*   Fy_body =  sin(δ)*Fx + cos(δ)*Fy                                   */
    /* Rear wheels are unsteered (δ = 0).                                   */
    /* ------------------------------------------------------------------ */

    float Fx_fl_body = cos_fl * Fx_fl - sin_fl * Fy_fl;
    float Fx_fr_body = cos_fr * Fx_fr - sin_fr * Fy_fr;
    float Fy_fl_body = sin_fl * Fx_fl + cos_fl * Fy_fl;
    float Fy_fr_body = sin_fr * Fx_fr + cos_fr * Fy_fr;

    float ax_tires = (Fx_fl_body + Fx_fr_body + Fx_rl + Fx_rr) / MASS_KG;
    s->ax = ax_tires - F_drag / MASS_KG;

    float vy_dot = (Fy_fl_body + Fy_fr_body + Fy_rl + Fy_rr) / MASS_KG
                   - vx * r;

    /* Yaw moment is the sum of each wheel-force's moment about the CG.       */
    /* A body-frame force (Fx_body, Fy_body) at corner (rx, ry) contributes   */
    /*   M = rx * Fy_body − ry * Fx_body.                                     */
    /* The Fx terms (×half-track) carry the torque-vectoring differential;    */
    /* the Fy terms (×lf/lr) carry the conventional cornering yaw moment.     */
    float r_dot = (
          (rx_fl * Fy_fl_body - ry_fl * Fx_fl_body)
        + (rx_fr * Fy_fr_body - ry_fr * Fx_fr_body)
        + (rx_rl * Fy_rl      - ry_rl * Fx_rl     )
        + (rx_rr * Fy_rr      - ry_rr * Fx_rr     )
        ) / YAW_INERTIA_KGM2;

    s->ay = vy_dot + vx * r;   /* lateral accel for G-G display */

    /* ------------------------------------------------------------------ */
    /* 8. Integrate states  (semi-implicit / symplectic Euler)              */
    /*                                                                     */
    /* Velocities are advanced here FIRST, then heading and position (step  */
    /* 9) are integrated from the just-updated yaw_rate / velocity / vy —    */
    /* not the start-of-tick values.  This semi-implicit ordering is what    */
    /* keeps the integrator stable on the stiff Pacejka tyre model at 100 Hz */
    /* (a fully-explicit scheme that propagated position from the old        */
    /* velocities would lag and could ring at the grip limit).              */
    /* ------------------------------------------------------------------ */

    s->vy       += vy_dot * dt;
    s->yaw_rate += r_dot  * dt;
    s->velocity += s->ax  * dt;

    /* First-order lag on the accelerations that drive load transfer.
     * alpha = dt / (tau + dt) is the discrete one-pole coefficient. */
    {
        float alpha_lp = dt / (LOAD_TRANSFER_TAU_S + dt);
        s->ax_filt += alpha_lp * (s->ax - s->ax_filt);
        s->ay_filt += alpha_lp * (s->ay - s->ay_filt);
    }

    /* Damp lateral motion at very low speed to avoid division-by-vx blowup */
    if (vx < 0.5f) {
        s->vy       *= 0.85f;
        s->yaw_rate *= 0.90f;
    }

    /* Yaw-rate safety net: r <= 1.5 * mu*g / vx.
     *
     * The per-wheel friction circle (step 6b) is now the PRIMARY grip limiter
     * and governs normal handling, so this clamp no longer needs to shape the
     * yaw response — it was previously doing double duty as a stand-in for the
     * missing combined-slip limit.  Widened to 1.5x the steady-state limit so
     * it only catches a numerical blow-up (e.g. an integration spike at the
     * grip edge) without cutting into yaw rates the tyre model legitimately
     * produces during transient rotation. */
    if (vx > 1.0f) {
        float r_max = 1.5f * (MU_TYRE * g) / vx;
        if (s->yaw_rate >  r_max) s->yaw_rate =  r_max;
        if (s->yaw_rate < -r_max) s->yaw_rate = -r_max;
    }

    if (s->velocity < 0.0f)        s->velocity = 0.0f;
    if (s->velocity > MAX_SPEED_MS) s->velocity = MAX_SPEED_MS;

    /* ------------------------------------------------------------------ */
    /* 9. Integrate heading and position                                    */
    /* ------------------------------------------------------------------ */

    s->heading += s->yaw_rate * dt;

    float cos_h = cosf(s->heading);
    float sin_h = sinf(s->heading);
    s->x += (s->velocity * cos_h - s->vy * sin_h) * dt;
    s->y += (s->velocity * sin_h + s->vy * cos_h) * dt;

    /* ------------------------------------------------------------------ */
    /* 10. Per-wheel RPM outputs (motor shaft frame)                        */
    /* ------------------------------------------------------------------ */

    s->wheelspeed[WHEEL_FL] = vx_fl / rpm2ms;
    s->wheelspeed[WHEEL_FR] = vx_fr / rpm2ms;
    s->wheelspeed[WHEEL_RL] = vx_rl / rpm2ms;
    s->wheelspeed[WHEEL_RR] = vx_rr / rpm2ms;

    /* ------------------------------------------------------------------ */
    /* 11. Body slip angle                                                  */
    /* ------------------------------------------------------------------ */

    s->slip_angle = (s->velocity > 0.5f)
                    ? atanf(s->vy / s->velocity)
                    : 0.0f;

    while (s->heading >  PI) s->heading -= 2.0f * PI;
    while (s->heading < -PI) s->heading += 2.0f * PI;
}
