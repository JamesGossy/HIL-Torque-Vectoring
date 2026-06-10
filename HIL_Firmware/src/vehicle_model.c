/*
 * M25 4-corner dynamic vehicle model. Per-wheel Pacejka tyres, aero downforce
 * and drag, load transfer and a per-wheel friction circle. Loaded outer tyres
 * carry most grip while inner tyres saturate first, which torque vectoring
 * exploits.
 */

#include "../include/vehicle_model.h"
#include "../../shared/load_transfer.h"
#include <math.h>

static const float PI = 3.14159265358979323846f;

/* Pacejka lateral force for one tyre, alpha = slip angle, Fz = normal load. */
static float pacejka_lat(float alpha, float Fz)
{
    /* k = -alpha maps our sign convention to the reference formula. */
    float k     = -alpha;
    float Bk    = TYRE_B * k;
    float shape = sinf(TYRE_C * atanf(Bk - TYRE_E * (Bk - atanf(Bk))));
    return TYRE_D * Fz * shape;
}

/* Reset all vehicle state to a stationary start pose. */
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
    s->steer_fl   = 0.0f;
    s->steer_fr   = 0.0f;
    s->ax         = 0.0f;
    s->ay         = 0.0f;
    s->ax_filt    = 0.0f;
    s->ay_filt    = 0.0f;
    for (int i = 0; i < 4; i++)
        s->wheelspeed[i] = 0.0f;
}

/* Advance the vehicle state one physics tick from the applied wheel torques. */
void vehicle_model_update(VehicleState *s, const WheelTorques *t, float dt)
{
    float vx = s->velocity;
    float vy = s->vy;
    float r  = s->yaw_rate;
    float g  = 9.81f;

    /* ---- steering geometry ---- */
    if (s->steering > 0.0f) { /* Ackermann per-wheel steer angles */
        s->steer_fl = INNER_STEERING_RATIO * s->steering;
        s->steer_fr = OUTER_STEERING_RATIO * s->steering;
    } else {
        s->steer_fl = OUTER_STEERING_RATIO * s->steering;
        s->steer_fr = INNER_STEERING_RATIO * s->steering;
    }

    /* ---- loads: aero + longitudinal + lateral transfer ---- */

    float F_drag = CDA * (0.5f * AIR_DENSITY * AERO_AREA * vx * vx);

    /* Lagged accelerations, not same-tick, so the model does not ring. */
    float Fz[4];
    load_transfer(vx, s->ax_filt, s->ay_filt, Fz);
    float Fz_fl = Fz[WHEEL_FL];
    float Fz_fr = Fz[WHEEL_FR];
    float Fz_rl = Fz[WHEEL_RL];
    float Fz_rr = Fz[WHEEL_RR];

    /* ---- slip angles ---- */

    /* 4-corner velocities, v_corner = v_CoG + omega x r_corner. */
    float half_sf = TRACK_WIDTH_FRONT_M * 0.5f;
    float half_sr = TRACK_WIDTH_REAR_M * 0.5f;

    /* Corner positions in vehicle frame, x forward, y left. */
    float rx_fl = CG_TO_FRONT_M, ry_fl = half_sf;
    float rx_fr = CG_TO_FRONT_M, ry_fr = -half_sf;
    float rx_rl = -CG_TO_REAR_M, ry_rl = half_sr;
    float rx_rr = -CG_TO_REAR_M, ry_rr = -half_sr;

    float vx_fl = vx - r * ry_fl, vy_fl = vy + r * rx_fl;
    float vx_fr = vx - r * ry_fr, vy_fr = vy + r * rx_fr;
    float vx_rl = vx - r * ry_rl, vy_rl = vy + r * rx_rl;
    float vx_rr = vx - r * ry_rr, vy_rr = vy + r * rx_rr;

    /* slip angles at each corner */
    float eps = 1e-5f;

    int stillstand = (vx * vx + vy * vy < 0.01f) && (fabsf(r) < 0.001f); /* zero slip below 0.1 m/s */

    float alpha_fl = 0.0f, alpha_fr = 0.0f;
    float alpha_rl = 0.0f, alpha_rr = 0.0f;

    if (!stillstand && vx > 0.5f) { /* slip = steer angle - velocity-vector angle */
        alpha_fl = s->steer_fl - atanf(vy_fl / ((fabsf(vx_fl) > eps) ? vx_fl : eps));
        alpha_fr = s->steer_fr - atanf(vy_fr / ((fabsf(vx_fr) > eps) ? vx_fr : eps));
        alpha_rl = -atanf(vy_rl / ((fabsf(vx_rl) > eps) ? vx_rl : eps));
        alpha_rr = -atanf(vy_rr / ((fabsf(vx_rr) > eps) ? vx_rr : eps));
    }

    /* ---- tyre forces and friction circle ---- */

    float Fy_fl = pacejka_lat(alpha_fl, Fz_fl); /* per-wheel lateral tyre forces */
    float Fy_fr = pacejka_lat(alpha_fr, Fz_fr);
    float Fy_rl = pacejka_lat(alpha_rl, Fz_rl);
    float Fy_rr = pacejka_lat(alpha_rr, Fz_rr);

    /* drive forces from wheel torques */
    float rpm2ms = WHEEL_RADIUS_M * 2.0f * PI / (GEAR_RATIO * 60.0f);

    float Fx_fl = GEAR_RATIO * t->fl / WHEEL_RADIUS_M; /* per-wheel drive force */
    float Fx_fr = GEAR_RATIO * t->fr / WHEEL_RADIUS_M;
    float Fx_rl = GEAR_RATIO * t->rl / WHEEL_RADIUS_M;
    float Fx_rr = GEAR_RATIO * t->rr / WHEEL_RADIUS_M;

    Fx_fl *= ((t->fl > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f; /* gate suppresses creep at standstill */
    Fx_fr *= ((t->fr > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;
    Fx_rl *= ((t->rl > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;
    Fx_rr *= ((t->rr > 0.5f) || (vx > 0.3f)) ? 1.0f : 0.0f;

    /* Friction circle: project each wheel's (Fx,Fy) back onto radius MU_GRIP*Fz so power mid-corner bleeds lateral grip. */
    {
        float Fmax_fl = MU_GRIP * Fz_fl;
        float Fmax_fr = MU_GRIP * Fz_fr;
        float Fmax_rl = MU_GRIP * Fz_rl;
        float Fmax_rr = MU_GRIP * Fz_rr;

        float c_fl = sqrtf(Fx_fl * Fx_fl + Fy_fl * Fy_fl);
        float c_fr = sqrtf(Fx_fr * Fx_fr + Fy_fr * Fy_fr);
        float c_rl = sqrtf(Fx_rl * Fx_rl + Fy_rl * Fy_rl);
        float c_rr = sqrtf(Fx_rr * Fx_rr + Fy_rr * Fy_rr);

        if (c_fl > Fmax_fl) {
            float k = Fmax_fl / c_fl;
            Fx_fl *= k;
            Fy_fl *= k;
        }
        if (c_fr > Fmax_fr) {
            float k = Fmax_fr / c_fr;
            Fx_fr *= k;
            Fy_fr *= k;
        }
        if (c_rl > Fmax_rl) {
            float k = Fmax_rl / c_rl;
            Fx_rl *= k;
            Fy_rl *= k;
        }
        if (c_rr > Fmax_rr) {
            float k = Fmax_rr / c_rr;
            Fx_rr *= k;
            Fy_rr *= k;
        }
    }

    float cos_fl = cosf(s->steer_fl), sin_fl = sinf(s->steer_fl);
    float cos_fr = cosf(s->steer_fr), sin_fr = sinf(s->steer_fr);

    /* ---- equations of motion ---- */

    /* Equations of motion. Front forces resolve into body frame by steer angle, rear wheels unsteered. */
    float Fx_fl_body = cos_fl * Fx_fl - sin_fl * Fy_fl;
    float Fx_fr_body = cos_fr * Fx_fr - sin_fr * Fy_fr;
    float Fy_fl_body = sin_fl * Fx_fl + cos_fl * Fy_fl;
    float Fy_fr_body = sin_fr * Fx_fr + cos_fr * Fy_fr;

    float ax_tires = (Fx_fl_body + Fx_fr_body + Fx_rl + Fx_rr) / MASS_KG;
    s->ax          = ax_tires - F_drag / MASS_KG;

    float vy_dot = (Fy_fl_body + Fy_fr_body + Fy_rl + Fy_rr) / MASS_KG - vx * r;

    /* Yaw moment, sum of each wheel moment about CG. Fx terms carry the TV differential. */
    float r_dot
        = ((rx_fl * Fy_fl_body - ry_fl * Fx_fl_body) + (rx_fr * Fy_fr_body - ry_fr * Fx_fr_body)
              + (rx_rl * Fy_rl - ry_rl * Fx_rl) + (rx_rr * Fy_rr - ry_rr * Fx_rr))
        / YAW_INERTIA_KGM2;

    s->ay = vy_dot + vx * r; /* lateral accel for G-G display */

    /* ---- integration and outputs ---- */

    /* Semi-implicit Euler, velocities first then position, keeps the stiff model stable at 100 Hz. */
    s->vy += vy_dot * dt;
    s->yaw_rate += r_dot * dt;
    s->velocity += s->ax * dt;

    { /* first-order lag on the accelerations that drive load transfer */
        float alpha_lp = dt / (LOAD_TRANSFER_TAU_S + dt);
        s->ax_filt += alpha_lp * (s->ax - s->ax_filt);
        s->ay_filt += alpha_lp * (s->ay - s->ay_filt);
    }

    if (vx < 0.5f) { /* damp lateral motion at low speed to avoid divide-by-vx blowup */
        s->vy *= 0.85f;
        s->yaw_rate *= 0.90f;
    }

    if (vx > 1.0f) { /* yaw-rate safety net, only catches numerical blow-ups */
        float r_max = 1.5f * (MU_TYRE * g) / vx;
        if (s->yaw_rate > r_max) s->yaw_rate = r_max;
        if (s->yaw_rate < -r_max) s->yaw_rate = -r_max;
    }

    if (s->velocity < 0.0f) s->velocity = 0.0f;
    if (s->velocity > MAX_SPEED_MS) s->velocity = MAX_SPEED_MS;

    s->heading += s->yaw_rate * dt; /* integrate heading and position */

    float cos_h = cosf(s->heading);
    float sin_h = sinf(s->heading);
    s->x += (s->velocity * cos_h - s->vy * sin_h) * dt;
    s->y += (s->velocity * sin_h + s->vy * cos_h) * dt;

    s->wheelspeed[WHEEL_FL] = vx_fl / rpm2ms; /* per-wheel RPM outputs */
    s->wheelspeed[WHEEL_FR] = vx_fr / rpm2ms;
    s->wheelspeed[WHEEL_RL] = vx_rl / rpm2ms;
    s->wheelspeed[WHEEL_RR] = vx_rr / rpm2ms;

    s->slip_angle = (s->velocity > 0.5f) ? atanf(s->vy / s->velocity) : 0.0f; /* body slip angle */

    /* wrap heading to +-pi */
    while (s->heading > PI)
        s->heading -= 2.0f * PI;
    while (s->heading < -PI)
        s->heading += 2.0f * PI;
}
