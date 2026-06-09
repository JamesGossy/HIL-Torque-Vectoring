#include "../include/torque_vectoring.h"
#include "../../shared/vehicle_config.h"
#include <math.h>

/*
 * torque_vectoring.c
 *
 * Model-based yaw-moment controller. It shifts torque from the inner to the
 * outer wheels so the car rotates at the rate the steering is asking for, no
 * more (oversteer) and no less (understeer). It has two stages:
 *
 *   A. Yaw moment. A model-based law produces the desired left-right torque
 *      differential ("bias"):
 *        1. Feedforward pre-loads the differential from the demanded yaw rate
 *           (known the instant the steering moves), so the moment is there as
 *           the car turns in and feedback only has to trim the rest.
 *        2. PI feedback. P trims the residual error; I erases the standing
 *           understeer a P-only loop would leave. The integrator freezes when
 *           the bias saturates so it cannot wind up.
 *        3. Derivative damping on the yaw error, to stop over-rotation on
 *           turn-in.
 *   B. Grip-aware distribution (grip_limits() + the bleed below). Each corner's
 *      grip-limited torque ceiling is estimated from a tyre/load model, then the
 *      driver's total torque is allocated by "bleeding" off those ceilings to
 *      realise the yaw moment first and the total second. This honours the
 *      differential up to what the tyres can actually deliver and naturally
 *      shifts authority to the loaded outer/rear corners - the production
 *      rules-based approach, replacing the old fixed symmetric split.
 *
 * The controller keeps a little internal state (the integrator, previous error,
 * and lagged accel estimates) in static variables. That is safe because the HIL
 * host calls this once per fixed-rate tick, and the state self-heals (anti-wound,
 * decays to zero). Gains come from shared/parameters_config.h.
 */

/* Geometry aliases from shared/vehicle_config.h. */
#define TV_WHEELBASE_M    WHEELBASE_M
#define TV_TRACK_WIDTH_M  TRACK_WIDTH_M
#define TV_WHEEL_RADIUS_M WHEEL_RADIUS_M

/* Front share of the differential, derived from the rear share. */
#define TV_FRONT_SHARE (1.0f - TV_REAR_SHARE)


/* Controller memory. File-scope so torque_vectoring_reset() can clear it. */
static float g_integ      = 0.0f; /* integral of yaw error */
static float g_prev_error = 0.0f; /* previous tick's yaw error */
static int g_primed       = 0;    /* false until the first call seeds */
static float g_prev_v     = 0.0f; /* previous tick's velocity, for ax estimate */
static float g_ax_filt    = 0.0f; /* lagged longitudinal accel estimate, m/s^2 */
static float g_ay_filt    = 0.0f; /* lagged lateral accel estimate, m/s^2 */

void torque_vectoring_reset(void)
{
    g_integ      = 0.0f;
    g_prev_error = 0.0f;
    g_primed     = 0;
    g_prev_v     = 0.0f;
    g_ax_filt    = 0.0f;
    g_ay_filt    = 0.0f;
}


/* Per-wheel grip-limited torque ceiling, mirroring the vehicle model's load
 * model (vehicle_model.c step 3) from sensor data alone, so the distributor can
 * be tyre-aware instead of only motor-aware. This is the MMS "tyre model": it
 * estimates each corner's normal load (static split + aero downforce + load
 * transfer) and converts the friction-circle force MU_GRIP*Fz into a motor
 * torque ceiling Fz*MU_GRIP*R/gear.
 *
 * Everything here is reconstructable inside the ECU boundary: the physical
 * constants live in shared/vehicle_config.h, lateral accel is ay = v*yaw_rate
 * (no accelerometer needed), and longitudinal accel is estimated from the
 * velocity trend. Both accels are first-order lagged with the same time
 * constant the model uses, so the estimate does not ring tick-to-tick. */
static void grip_limits(const SensorData *sensors, float grip[4])
{
    float v = sensors->velocity;

    /* Lateral accel straight from sensors: ay = v * r. */
    float ay = v * sensors->yaw_rate;

    /* Longitudinal accel from the velocity trend (seeded on the first call). */
    float ax = 0.0f;
    if (g_prev_v > 0.0f) ax = (v - g_prev_v) / CONTROL_DT_S;
    g_prev_v = v;

    /* First-order lag, matching LOAD_TRANSFER_TAU_S in the model. */
    float alpha = CONTROL_DT_S / (LOAD_TRANSFER_TAU_S + CONTROL_DT_S);
    g_ax_filt += alpha * (ax - g_ax_filt);
    g_ay_filt += alpha * (ay - g_ay_filt);

    const float g = 9.81f;

    /* Aero downforce (50/50 split), same form as the model. */
    float q           = 0.5f * AIR_DENSITY * AERO_AREA * v * v;
    float F_downforce = CLA * q;

    float Fz_front_axle = MASS_KG * g * (CG_TO_REAR_M / WHEELBASE_M) + F_downforce * 0.5f;
    float Fz_rear_axle  = MASS_KG * g * (CG_TO_FRONT_M / WHEELBASE_M) + F_downforce * 0.5f;

    float dFz_long = g_ax_filt * MASS_KG * CG_HEIGHT_M / WHEELBASE_M;
    Fz_front_axle -= dFz_long;
    Fz_rear_axle += dFz_long;
    if (Fz_front_axle < 50.0f) Fz_front_axle = 50.0f;
    if (Fz_rear_axle < 50.0f) Fz_rear_axle = 50.0f;

    float m_front       = MASS_KG * (CG_TO_REAR_M / WHEELBASE_M);
    float m_rear        = MASS_KG * (CG_TO_FRONT_M / WHEELBASE_M);
    float dFz_lat_front = m_front * g_ay_filt * CG_HEIGHT_M / TRACK_WIDTH_FRONT_M;
    float dFz_lat_rear  = m_rear * g_ay_filt * CG_HEIGHT_M / TRACK_WIDTH_REAR_M;

    float Fz[4];
    Fz[WHEEL_FL] = 0.5f * Fz_front_axle - dFz_lat_front;
    Fz[WHEEL_FR] = 0.5f * Fz_front_axle + dFz_lat_front;
    Fz[WHEEL_RL] = 0.5f * Fz_rear_axle - dFz_lat_rear;
    Fz[WHEEL_RR] = 0.5f * Fz_rear_axle + dFz_lat_rear;

    /* Convert each corner's friction-circle force MU_GRIP*Fz to a motor-torque
     * ceiling Fz*MU_GRIP*R/gear, capped at the motor peak. We do NOT subtract the
     * lateral force here: the driver's throttle controller already backs total
     * torque off under lateral load (its traction-circle cut), so subtracting Fy
     * again would double-count and choke corner-exit drive. The grip ceiling's
     * job is to set the RELATIVE authority of each corner for the distributor -
     * the loaded outer/rear tyres carry more, the unloaded inner front less -
     * which is exactly the rear-and-outward shift the old fixed split only
     * approximated with TV_REAR_SHARE. */
    for (int i = 0; i < 4; i++) {
        if (Fz[i] < 25.0f) Fz[i] = 25.0f;
        float t = MU_GRIP * Fz[i] * TV_WHEEL_RADIUS_M / GEAR_RATIO;
        if (t > MAX_MOTOR_TORQUE_NM) t = MAX_MOTOR_TORQUE_NM;
        if (t < 1.0f) t = 1.0f;
        grip[i] = t;
    }
}

void torque_vectoring_update(
    const SensorData *sensors, float driver_torque, float kp_yaw, WheelTorques *out)
{
    /* Step 1: desired yaw rate. The v^2 understeer term bends the reference down
     * to the yaw rate the car can actually reach:
     *   r = v * tan(delta) / (L + K_us * v^2). */
    /* The desired yaw rate is intentionally computed from the INSTANTANEOUS
     * steering angle and used un-filtered: the feedforward must put the yaw
     * moment in the instant the steering moves (see Step 4), so the response is
     * there before any error develops. The signal looks jittery on a plot
     * because the path tracker moves the steering fast and tan() amplifies near
     * lock, and the car's yaw rate cannot follow those spikes - but it is NOT
     * meant to: the spikes are the feedforward reacting early, not a setpoint to
     * track. Low-pass-filtering this reference was tried and measurably hurt the
     * car (it delays the moment, so the car understeers into corners). */
    float desired_yaw_rate = 0.0f;
    if (sensors->velocity > 0.5f) {
        float v          = sensors->velocity;
        desired_yaw_rate = v * tanf(sensors->steering_angle) / (TV_WHEELBASE_M + TV_K_US * v * v);
    }

    /* Step 1b: fuse the IMU yaw rate with a wheel-speed estimate. The wheel
     * estimate is r_wheels = (v_right - v_left) / track, averaged front/rear.
     * It corroborates the gyro but is down-weighted as it fails under slip. */
    float v_fl = sensors->wheel_speed[WHEEL_FL] * TV_WHEEL_RADIUS_M;
    float v_fr = sensors->wheel_speed[WHEEL_FR] * TV_WHEEL_RADIUS_M;
    float v_rl = sensors->wheel_speed[WHEEL_RL] * TV_WHEEL_RADIUS_M;
    float v_rr = sensors->wheel_speed[WHEEL_RR] * TV_WHEEL_RADIUS_M;

    float r_front  = (v_fr - v_fl) / TV_TRACK_WIDTH_M;
    float r_rear   = (v_rr - v_rl) / TV_TRACK_WIDTH_M;
    float r_wheels = 0.5f * (r_front + r_rear);

    float yaw_rate_est
        = (1.0f - TV_WHEEL_YAW_TRUST) * sensors->yaw_rate + TV_WHEEL_YAW_TRUST * r_wheels;

    /* Step 2: yaw error, with a deadband so the bias does not chatter on noise. */
    float yaw_error = desired_yaw_rate - yaw_rate_est;
    if (yaw_error > TV_YAW_DEADBAND)
        yaw_error -= TV_YAW_DEADBAND;
    else if (yaw_error < -TV_YAW_DEADBAND)
        yaw_error += TV_YAW_DEADBAND;
    else
        yaw_error = 0.0f;

    /* Seed the derivative memory on the first call so it does not see a spike. */
    if (!g_primed) {
        g_prev_error = yaw_error;
        g_primed     = 1;
    }

    /* Step 3: scale the master gain inversely with speed so the yaw response
     * stays consistent across the speed range. Capped near standstill. */
    float effective_kp = kp_yaw;
    if (sensors->velocity > 2.0f) {
        effective_kp = kp_yaw * (TV_SPEED_REF_MS / sensors->velocity);
        if (effective_kp > kp_yaw * 3.0f) effective_kp = kp_yaw * 3.0f;
    }

    /* Step 4: feedforward. Pre-loads the differential from the cornering demand
     * so the moment is there before any error develops. */
    float ff = 0.0f;
    if (kp_yaw > 0.0f && sensors->velocity > 2.0f)
        ff = TV_KFF * desired_yaw_rate * sensors->velocity;

    /* Step 5: PID feedback. P trims the residual, I erases standing understeer,
     * D damps turn-in. Gain = 0 disables the whole FF+feedback path. */
    float p_term = effective_kp * yaw_error;

    float d_error = (yaw_error - g_prev_error) / CONTROL_DT_S;
    float d_term  = (TV_KD_FRAC * effective_kp) * d_error;
    g_prev_error  = yaw_error;

    /* Provisional integral update (may be rolled back by anti-windup below). */
    float ki         = TV_KI_FRAC * effective_kp;
    float integ_next = g_integ + yaw_error * CONTROL_DT_S;
    /* Hard cap the integral contribution to bias. */
    float i_contrib = ki * integ_next;
    if (i_contrib > TV_I_MAX_NM) {
        integ_next = TV_I_MAX_NM / (ki > 1e-6f ? ki : 1e-6f);
    }
    if (i_contrib < -TV_I_MAX_NM) {
        integ_next = -TV_I_MAX_NM / (ki > 1e-6f ? ki : 1e-6f);
    }
    float i_term = ki * integ_next;

    float bias = 0.0f;
    if (kp_yaw > 0.0f) bias = ff + p_term + i_term + d_term;

    /* Clamp total bias to half the motor peak. This lets the inner wheel go
     * into regen while the outer drives, for a stronger moment. */
    float max_bias   = MAX_MOTOR_TORQUE_NM * 0.5f;
    int bias_clamped = 0;
    if (bias > max_bias) {
        bias         = max_bias;
        bias_clamped = 1;
    }
    if (bias < -max_bias) {
        bias         = -max_bias;
        bias_clamped = 1;
    }

    /* Anti-windup: only commit the integrator when the bias is not clamped, or
     * the error is driving it back out of saturation. Otherwise hold it. */
    float integ_prev = g_integ;
    if (kp_yaw <= 0.0f) {
        g_integ = 0.0f;
    } else if (!bias_clamped || (bias >= max_bias && yaw_error < 0.0f)
        || (bias <= -max_bias && yaw_error > 0.0f)) {
        g_integ = integ_next;
    }

    /* Step 6: grip-aware bleed distribution (MMS rules-based philosophy). Instead
     * of adding a symmetric bias to a base torque and clamping at the motor peak,
     * we start each side at its tyre grip ceiling and bleed torque off to hit the
     * yaw moment first, then the driver's total - so the differential is honoured
     * up to what the tyres can actually deliver, and grip-limited corners shift
     * authority rearward/outward instead of just clipping.
     *
     * Drive and regen share the same logic on the magnitude: the friction circle
     * is symmetric in Fx sign, so we distribute |driver_torque| against the grip
     * ceilings and restore the sign at the end (TV still acts on regen). */
    float grip[4];
    grip_limits(sensors, grip);

    float left_lim  = grip[WHEEL_FL] + grip[WHEEL_RL];
    float right_lim = grip[WHEEL_FR] + grip[WHEEL_RR];

    /* The yaw controller's bias IS the desired left-right differential (Nm of
     * motor torque). Positive bias adds to the right (outer in a left turn). */
    float desired_diff = bias;

    /* Total magnitude the driver is asking for, capped at the available grip. */
    float total_target = fabsf(driver_torque);
    float sum_lim      = left_lim + right_lim;
    if (total_target > sum_lim) total_target = sum_lim;

    /* Start from the ceilings; bleed the side that must give up torque to realise
     * the differential left - right = -desired_diff (positive diff -> more right). */
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

    /* Bleed both sides evenly down to the driver's total, preserving the
     * differential. If one side floors, take the remainder from the other. */
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

    /* Step 7: allocate each side front/rear in proportion to that wheel's grip
     * ceiling (equal tyre utilisation), the way MMS splits L/R back to corners. */
    float fl = (left_lim > 1e-3f) ? left * (grip[WHEEL_FL] / left_lim) : 0.0f;
    float rl = (left_lim > 1e-3f) ? left * (grip[WHEEL_RL] / left_lim) : 0.0f;
    float fr = (right_lim > 1e-3f) ? right * (grip[WHEEL_FR] / right_lim) : 0.0f;
    float rr = (right_lim > 1e-3f) ? right * (grip[WHEEL_RR] / right_lim) : 0.0f;

    /* Step 8: restore the driver's sign (regen = negative) and write out. */
    float sign = (driver_torque < 0.0f) ? -1.0f : 1.0f;
    out->fl    = sign * fl;
    out->fr    = sign * fr;
    out->rl    = sign * rl;
    out->rr    = sign * rr;

    /* Anti-windup: if the bleed could not realise the demanded differential
     * (a side floored) while the error still drove the bias the same way, roll
     * the integrator back so it cannot wind up against grip it does not have. */
    if (bleed_saturated && kp_yaw > 0.0f) {
        if ((bias >= 0.0f && yaw_error > 0.0f) || (bias < 0.0f && yaw_error < 0.0f))
            g_integ = integ_prev;
    }
}
