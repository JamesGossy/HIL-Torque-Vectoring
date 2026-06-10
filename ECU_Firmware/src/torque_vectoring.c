#include "../include/torque_vectoring.h"
#include "../../shared/vehicle_config.h"
#include "../../shared/tunables.h"
#include <math.h>

/* Yaw-moment torque vectoring controller for the ECU. Shifts torque from inner
   to outer wheels so the car rotates at the rate the steering asks for. A
   feedforward plus PID yaw loop sets a left-right differential, then a
   grip-aware bleed distributes the driver torque across the four corners. */

#define TV_WHEELBASE_M    WHEELBASE_M
#define TV_WHEEL_RADIUS_M WHEEL_RADIUS_M


static float g_integ      = 0.0f;
static float g_prev_error = 0.0f;
static int g_primed       = 0;
static float g_prev_v     = 0.0f;
static float g_ax_filt    = 0.0f;
static float g_ay_filt    = 0.0f;

/* Clears all controller state between independent runs. */
void torque_vectoring_reset(void)
{
    g_integ      = 0.0f;
    g_prev_error = 0.0f;
    g_primed     = 0;
    g_prev_v     = 0.0f;
    g_ax_filt    = 0.0f;
    g_ay_filt    = 0.0f;
}


/* Estimates each corner's grip-limited motor torque ceiling from sensor data. */
static void grip_limits(const SensorData *sensors, float grip[4])
{
    float v = sensors->velocity;

    // accelerations from sensors, filtered to match the load-transfer lag
    float ay = v * sensors->yaw_rate; // lateral accel straight from sensors

    float ax = 0.0f; // longitudinal accel from the velocity trend
    if (g_prev_v > 0.0f) ax = (v - g_prev_v) / CONTROL_DT_S;
    g_prev_v = v;

    float alpha = CONTROL_DT_S / (LOAD_TRANSFER_TAU_S + CONTROL_DT_S); // lag matches the model
    g_ax_filt += alpha * (ax - g_ax_filt);
    g_ay_filt += alpha * (ay - g_ay_filt);

    const float g = 9.81f;

    // static axle loads plus aero downforce
    float q           = 0.5f * AIR_DENSITY * AERO_AREA * v * v; // aero downforce, 50/50 split
    float F_downforce = CLA * q;

    float Fz_front_axle = MASS_KG * g * (CG_TO_REAR_M / WHEELBASE_M) + F_downforce * 0.5f;
    float Fz_rear_axle  = MASS_KG * g * (CG_TO_FRONT_M / WHEELBASE_M) + F_downforce * 0.5f;

    // longitudinal weight transfer between axles
    float dFz_long = g_ax_filt * MASS_KG * CG_HEIGHT_M / WHEELBASE_M;
    Fz_front_axle -= dFz_long;
    Fz_rear_axle += dFz_long;
    if (Fz_front_axle < 50.0f) Fz_front_axle = 50.0f;
    if (Fz_rear_axle < 50.0f) Fz_rear_axle = 50.0f;

    // lateral weight transfer at each axle
    float m_front       = MASS_KG * (CG_TO_REAR_M / WHEELBASE_M);
    float m_rear        = MASS_KG * (CG_TO_FRONT_M / WHEELBASE_M);
    float dFz_lat_front = m_front * g_ay_filt * CG_HEIGHT_M / TRACK_WIDTH_FRONT_M;
    float dFz_lat_rear  = m_rear * g_ay_filt * CG_HEIGHT_M / TRACK_WIDTH_REAR_M;

    float Fz[4];
    Fz[WHEEL_FL] = 0.5f * Fz_front_axle - dFz_lat_front;
    Fz[WHEEL_FR] = 0.5f * Fz_front_axle + dFz_lat_front;
    Fz[WHEEL_RL] = 0.5f * Fz_rear_axle - dFz_lat_rear;
    Fz[WHEEL_RR] = 0.5f * Fz_rear_axle + dFz_lat_rear;

    // do not subtract Fy here, the throttle controller already cuts for lateral load (double-count)
    for (int i = 0; i < 4; i++) {
        if (Fz[i] < 25.0f) Fz[i] = 25.0f;
        float t = MU_GRIP * Fz[i] * TV_WHEEL_RADIUS_M / GEAR_RATIO;
        if (t > MAX_MOTOR_TORQUE_NM) t = MAX_MOTOR_TORQUE_NM;
        if (t < 1.0f) t = 1.0f;
        grip[i] = t;
    }
}

/* Runs the yaw loop and grip-aware bleed to set the four wheel torques. */
void torque_vectoring_update(
    const SensorData *sensors, float driver_torque, float kp_yaw, WheelTorques *out)
{
    // 1. desired yaw rate from instantaneous steering, kept unfiltered (filtering it hurt the car)
    float desired_yaw_rate = 0.0f;
    if (sensors->velocity > 0.5f) {
        float v          = sensors->velocity;
        desired_yaw_rate = v * tanf(sensors->steering_angle) / (TV_WHEELBASE_M + g_TV_K_US * v * v);
    }

    // 2. measured yaw rate from the gyro
    float yaw_rate_est = sensors->yaw_rate;

    float yaw_error = desired_yaw_rate - yaw_rate_est; // deadband below stops chatter on noise
    if (yaw_error > g_TV_YAW_DEADBAND)
        yaw_error -= g_TV_YAW_DEADBAND;
    else if (yaw_error < -g_TV_YAW_DEADBAND)
        yaw_error += g_TV_YAW_DEADBAND;
    else
        yaw_error = 0.0f;

    if (!g_primed) { // seed derivative memory on first call so it sees no spike
        g_prev_error = yaw_error;
        g_primed     = 1;
    }

    /* ---- yaw loop ---- */

    // 3. feedforward + PID to get the left-right torque bias
    float effective_kp = kp_yaw;

    float ff = 0.0f; // feedforward pre-loads the differential before any error develops
    if (kp_yaw > 0.0f && sensors->velocity > 2.0f)
        ff = (g_TV_KFF_FRAC * kp_yaw) * desired_yaw_rate * sensors->velocity;

    float p_term = effective_kp * yaw_error; // gain 0 disables the whole FF+feedback path

    float d_error = (yaw_error - g_prev_error) / CONTROL_DT_S;
    float d_term  = (g_TV_KD_FRAC * effective_kp) * d_error;
    g_prev_error  = yaw_error;

    float ki         = g_TV_KI_FRAC * effective_kp;
    float integ_next = g_integ + yaw_error * CONTROL_DT_S; // may be rolled back by anti-windup below
    float i_max     = g_TV_I_MAX_FRAC * MAX_MOTOR_TORQUE_NM; // cap scales with motor authority
    float i_contrib = ki * integ_next;
    if (i_contrib > i_max) {
        integ_next = i_max / (ki > 1e-6f ? ki : 1e-6f);
    }
    if (i_contrib < -i_max) {
        integ_next = -i_max / (ki > 1e-6f ? ki : 1e-6f);
    }
    float i_term = ki * integ_next;

    float bias = 0.0f;
    if (kp_yaw > 0.0f) bias = ff + p_term + i_term + d_term;

    float max_bias   = MAX_MOTOR_TORQUE_NM * 0.5f; // lets inner regen while outer drives
    int bias_clamped = 0;
    if (bias > max_bias) {
        bias         = max_bias;
        bias_clamped = 1;
    }
    if (bias < -max_bias) {
        bias         = -max_bias;
        bias_clamped = 1;
    }

    // anti-windup: commit integrator only when not clamped or error drives it out of saturation
    float integ_prev = g_integ;
    if (kp_yaw <= 0.0f) {
        g_integ = 0.0f;
    } else if (!bias_clamped || (bias >= max_bias && yaw_error < 0.0f)
        || (bias <= -max_bias && yaw_error > 0.0f)) {
        g_integ = integ_next;
    }

    /* ---- bleed distributor ---- */

    // 4. grip-aware bleed: distribute |driver_torque| against grip ceilings, sign restored at the end
    float grip[4];
    grip_limits(sensors, grip);

    float left_lim  = grip[WHEEL_FL] + grip[WHEEL_RL];
    float right_lim = grip[WHEEL_FR] + grip[WHEEL_RR];

    float desired_diff = bias; // positive bias adds to the right (outer in a left turn)

    float total_target = fabsf(driver_torque); // capped at available grip
    float sum_lim      = left_lim + right_lim;
    if (total_target > sum_lim) total_target = sum_lim;

    // bleed the side that must give up torque to realise left - right = -desired_diff
    float left          = left_lim;
    float right         = right_lim;
    float ym_delta      = (left_lim - right_lim) - (-desired_diff);
    int bleed_saturated = 0;
    if (ym_delta > 0.0f) {
        left = left_lim - ym_delta;
        if (left < 0.0f) {
            left            = 0.0f;
            bleed_saturated = 1;
        }
    } else {
        right = right_lim + ym_delta;
        if (right < 0.0f) {
            right           = 0.0f;
            bleed_saturated = 1;
        }
    }

    // bleed both sides evenly to the driver total, if one floors take remainder from the other
    float over = (left + right) - total_target;
    if (over > 0.0f) {
        float new_left  = left - over * 0.5f;
        float new_right = right - over * 0.5f;
        if (new_left < 0.0f) {
            new_right += new_left;
            new_left = 0.0f;
        }
        if (new_right < 0.0f) {
            new_left += new_right;
            new_right = 0.0f;
        }
        left  = (new_left > 0.0f) ? new_left : 0.0f;
        right = (new_right > 0.0f) ? new_right : 0.0f;
    }

    // split each side front/rear in proportion to wheel grip ceiling (equal tyre utilisation)
    float fl = (left_lim > 1e-3f) ? left * (grip[WHEEL_FL] / left_lim) : 0.0f;
    float rl = (left_lim > 1e-3f) ? left * (grip[WHEEL_RL] / left_lim) : 0.0f;
    float fr = (right_lim > 1e-3f) ? right * (grip[WHEEL_FR] / right_lim) : 0.0f;
    float rr = (right_lim > 1e-3f) ? right * (grip[WHEEL_RR] / right_lim) : 0.0f;

    float sign = (driver_torque < 0.0f) ? -1.0f : 1.0f; // restore driver sign, regen negative
    out->fl    = sign * fl;
    out->fr    = sign * fr;
    out->rl    = sign * rl;
    out->rr    = sign * rr;

    // anti-windup: if the bleed floored while error still drove the bias, roll the integrator back
    if (bleed_saturated && kp_yaw > 0.0f) {
        if ((bias >= 0.0f && yaw_error > 0.0f) || (bias < 0.0f && yaw_error < 0.0f))
            g_integ = integ_prev;
    }
}
