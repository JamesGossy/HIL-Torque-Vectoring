#include "../include/torque_vectoring.h"
#include "../../shared/vehicle_config.h"
#include <math.h>

/*
 * torque_vectoring.c
 *
 * Model-based yaw-moment controller. It shifts torque from the inner to the
 * outer wheels of each axle so the car rotates at the rate the steering is
 * asking for, no more (oversteer) and no less (understeer). It has four parts:
 *
 *   1. Feedforward. Pre-loads the differential from the demanded yaw rate
 *      (known the instant the steering moves), so the moment is there as the
 *      car turns in and feedback only has to trim the rest.
 *   2. PI feedback. P trims the residual error; I erases the standing
 *      understeer a P-only loop would leave. The integrator freezes when the
 *      bias saturates so it cannot wind up.
 *   3. Derivative damping on the yaw error, to stop over-rotation on turn-in.
 *   4. Rear-biased split. The rear axle gets the larger share of the
 *      differential because its tyres have more spare grip in a corner.
 *
 * The controller keeps a little internal state (the integrator and previous
 * error) in static variables. That is safe because the HIL host calls this
 * once per fixed-rate tick, and the state self-heals (anti-wound, decays to
 * zero). Gains come from shared/parameters_config.h.
 */

/* Geometry aliases from shared/vehicle_config.h. */
#define TV_WHEELBASE_M    WHEELBASE_M
#define TV_TRACK_WIDTH_M  TRACK_WIDTH_M
#define TV_WHEEL_RADIUS_M WHEEL_RADIUS_M

/* Front share of the differential, derived from the rear share. */
#define TV_FRONT_SHARE (1.0f - TV_REAR_SHARE)


/* Clamp one axle's left/right torques to the motor limits while preserving the
 * differential (left - right), which carries the yaw moment. If one side
 * exceeds the peak, the excess is shifted onto the other side before clamping,
 * so a saturated outer wheel does not silently collapse the differential.
 * Returns 1 if either side hit a hard limit. */
static int clamp_axle_preserving(float left, float right, float *left_out, float *right_out)
{
    int saturated = 0;

    if (left > MAX_MOTOR_TORQUE_NM) {
        right -= (left - MAX_MOTOR_TORQUE_NM);
        left = MAX_MOTOR_TORQUE_NM;
    } else if (right > MAX_MOTOR_TORQUE_NM) {
        left -= (right - MAX_MOTOR_TORQUE_NM);
        right = MAX_MOTOR_TORQUE_NM;
    }
    if (left < MIN_MOTOR_TORQUE_NM) {
        left      = MIN_MOTOR_TORQUE_NM;
        saturated = 1;
    }
    if (left > MAX_MOTOR_TORQUE_NM) {
        left      = MAX_MOTOR_TORQUE_NM;
        saturated = 1;
    }
    if (right < MIN_MOTOR_TORQUE_NM) {
        right     = MIN_MOTOR_TORQUE_NM;
        saturated = 1;
    }
    if (right > MAX_MOTOR_TORQUE_NM) {
        right     = MAX_MOTOR_TORQUE_NM;
        saturated = 1;
    }

    *left_out  = left;
    *right_out = right;
    return saturated;
}


/* Controller memory. File-scope so torque_vectoring_reset() can clear it. */
static float g_integ      = 0.0f; /* integral of yaw error */
static float g_prev_error = 0.0f; /* previous tick's yaw error */
static int g_primed       = 0;    /* false until the first call seeds */

void torque_vectoring_reset(void)
{
    g_integ      = 0.0f;
    g_prev_error = 0.0f;
    g_primed     = 0;
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

    /* Step 6: base torque per wheel. driver_torque is total motor torque; split
     * it four ways. Do not pre-multiply by the gear ratio (the model does that). */
    float base_per_wheel = driver_torque * 0.25f;

    /* Step 7: split the differential front/rear. Positive steering is a left
     * turn, so the right wheels are outer and positive bias adds to the right.
     * The shares are weighted by 2x so they redistribute the bias around a mean
     * of 1.0: a rear share moves authority rearward without losing total moment
     * (TV_REAR_SHARE = 0.5 gives the symmetric case). */
    float bias_front = bias * (2.0f * TV_FRONT_SHARE);
    float bias_rear  = bias * (2.0f * TV_REAR_SHARE);

    /* Step 8: clamp to motor limits while preserving the differential, so a
     * saturated outer wheel does not collapse the yaw moment. */
    int sat_f = clamp_axle_preserving(
        base_per_wheel - bias_front * 0.5f, base_per_wheel + bias_front * 0.5f, &out->fl, &out->fr);
    int sat_r = clamp_axle_preserving(
        base_per_wheel - bias_rear * 0.5f, base_per_wheel + bias_rear * 0.5f, &out->rl, &out->rr);

    /* If a motor physically saturated while the error still drove the bias the
     * same way, roll the integrator back so it cannot wind up. */
    if ((sat_f || sat_r) && kp_yaw > 0.0f) {
        if ((bias >= 0.0f && yaw_error > 0.0f) || (bias < 0.0f && yaw_error < 0.0f))
            g_integ = integ_prev;
    }
}
