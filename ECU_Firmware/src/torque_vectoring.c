#include "../include/torque_vectoring.h"
#include "../../shared/vehicle_config.h"
#include <math.h>

/*
 * torque_vectoring.c
 *
 * The ECU torque-vectoring algorithm.
 *
 * Overview
 * --------
 * This is a model-based yaw-moment controller.  It decides how much torque to
 * shift from the inner to the outer wheels of each axle so the car rotates at
 * the rate the driver's steering is asking for — no more (oversteer), no less
 * (understeer).  Compared with a plain proportional yaw-rate controller it adds
 * four things, each of which the headless lap evaluator showed to matter:
 *
 *   1. FEEDFORWARD.  A pure feedback controller only acts once an error has
 *      already opened up, so it is always one step behind the corner.  We
 *      pre-load the differential from the *demanded* yaw rate (a quantity we
 *      know the instant the steering moves), so the yaw moment is already there
 *      as the car turns in.  Feedback then only has to trim the residual.
 *
 *   2. PI FEEDBACK with anti-windup.  Proportional alone leaves a standing
 *      yaw-rate error (the car settles understeering slightly forever).  A small
 *      integral term erases that steady-state error.  The integrator is frozen
 *      whenever the bias is saturated, so it cannot wind up against the clamp.
 *
 *   3. DERIVATIVE damping.  A derivative term on the yaw-rate error damps the
 *      turn-in transient, stopping the controller from over-rotating the car the
 *      moment it starts to respond — the classic source of corner-entry snap.
 *
 *   4. GRIP-AWARE, REAR-BIASED SPLIT.  The yaw moment is produced more
 *      cheaply at the rear axle (longer moment arm to the relevant tyres is not
 *      the point — it is that the rear tyres carry less steering-induced lateral
 *      demand, so they have more longitudinal budget to spend on the
 *      differential).  We send the larger share of the differential to the rear
 *      and a smaller share to the front, and we scale the whole effort down with
 *      speed so the moment authority stays consistent across the speed range.
 *
 * The controller keeps a small amount of internal state (the integrator and the
 * previous error) in static variables.  This is safe here because the HIL host
 * calls this function exactly once per fixed-rate control tick — see main.c.
 * The state is self-healing (it decays toward zero and is anti-wound), so a
 * missed or duplicated call cannot make it diverge.
 */

/* Vehicle geometry — sourced from shared/vehicle_config.h. */
#define TV_WHEELBASE_M    WHEELBASE_M
#define TV_TRACK_WIDTH_M  TRACK_WIDTH_M
#define TV_WHEEL_RADIUS_M WHEEL_RADIUS_M

/* Motor-to-wheel gear ratio, kept for reference only.  The ECU works entirely
 * in MOTOR torque (both driver_torque in and the WheelTorques out); the vehicle
 * model is what applies the gear ratio to get wheel force.  Do NOT multiply the
 * ECU outputs by this -- that double-counts the ratio and saturates the motors. */
#define TV_GEAR_RATIO   GEAR_RATIO

/* Weight on the wheel-speed-derived yaw estimate when fused with the IMU.
 * 0 = trust IMU only, 1 = trust wheel speeds only.  The wheel-speed estimate
 * is noise-free but degrades under wheel slip/scrub, so we keep the IMU as the
 * primary source and use the wheel-speed channel as a corroborating signal. */
#define TV_WHEEL_YAW_TRUST  0.25f

/* Fixed control period, s.  The HIL host runs the loop at 100 Hz (DT = 0.01).
 * The integral and derivative terms need a time base; using the known fixed
 * period keeps the controller stateless w.r.t. timing (no dt threaded in). */
#define TV_DT_S            0.01f

/* --- Feedback gains (relative to the runtime kp_yaw the user tunes) ---
 *
 * kp_yaw is the proportional gain in Nm of bias per rad/s of yaw error and is
 * the master knob ([ and ] at runtime).  Ki and Kd are expressed as fractions
 * of it so the whole loop scales together when the user retunes Kp.
 *
 *   Ki: integral gain    = TV_KI_FRAC * kp_yaw   (per second)
 *   Kd: derivative gain  = TV_KD_FRAC * kp_yaw   (per (rad/s)/s)
 */
#ifndef TV_KI_FRAC
#define TV_KI_FRAC         2.5f    /* erases steady-state understeer */
#endif
#ifndef TV_KD_FRAC
#define TV_KD_FRAC         0.05f   /* damps the turn-in transient    */
#endif

/* Feedforward gain: bias (Nm) per unit of (desired_yaw_rate * v).
 * desired_yaw_rate*v has units (rad/s)*(m/s); this constant converts the
 * cornering "demand" into a pre-loaded torque differential so the moment is
 * present before any error develops.  Tuned against the lap evaluator. */
#ifndef TV_KFF
#define TV_KFF             12.0f
#endif

/* Integrator clamp (Nm of bias contributed by the I term alone).  A hard cap in
 * addition to the freeze-on-saturation anti-windup, so a long sustained error
 * still cannot let the I term dominate the P/FF response. */
#define TV_I_MAX_NM        12.0f

/* TV_REAR_SHARE / TV_FRONT_SHARE (front/rear split of the yaw-moment
 * differential) are defined in torque_vectoring.h as part of the tuning
 * surface. */
#define TV_FRONT_SHARE     (1.0f - TV_REAR_SHARE)


/* Clamp one axle's left/right torques to the motor envelope while PRESERVING
 * the commanded differential (left - right), which is what carries the yaw
 * moment.  If one side exceeds the peak, the excess is shifted onto the other
 * side (into regen if needed) before the hard clamp, so a saturated outer wheel
 * no longer silently collapses the differential.  Returns 1 if either side hit
 * a hard rail (used by the caller to freeze the integrator). */
static int clamp_axle_preserving(float left, float right,
                                 float *left_out, float *right_out)
{
    int saturated = 0;

    if (left > MAX_MOTOR_TORQUE_NM) {
        right -= (left - MAX_MOTOR_TORQUE_NM);
        left   = MAX_MOTOR_TORQUE_NM;
    } else if (right > MAX_MOTOR_TORQUE_NM) {
        left -= (right - MAX_MOTOR_TORQUE_NM);
        right = MAX_MOTOR_TORQUE_NM;
    }
    if (left  < MIN_MOTOR_TORQUE_NM) { left  = MIN_MOTOR_TORQUE_NM; saturated = 1; }
    if (left  > MAX_MOTOR_TORQUE_NM) { left  = MAX_MOTOR_TORQUE_NM; saturated = 1; }
    if (right < MIN_MOTOR_TORQUE_NM) { right = MIN_MOTOR_TORQUE_NM; saturated = 1; }
    if (right > MAX_MOTOR_TORQUE_NM) { right = MAX_MOTOR_TORQUE_NM; saturated = 1; }

    *left_out  = left;
    *right_out = right;
    return saturated;
}


/* Controller memory.  File-scope (rather than function-static) so
 * torque_vectoring_reset() can clear it.  See the file header for why static
 * state is safe in this single-caller, fixed-rate context. */
static float g_integ      = 0.0f;   /* integral of yaw error (rad)        */
static float g_prev_error = 0.0f;   /* previous tick's yaw error (rad/s)  */
static int   g_primed     = 0;      /* false until the first call seeds    */

void torque_vectoring_reset(void)
{
    g_integ      = 0.0f;
    g_prev_error = 0.0f;
    g_primed     = 0;
}

void torque_vectoring_update(const SensorData *sensors,
                             float             driver_torque,
                             float             kp_yaw,
                             WheelTorques     *out)
{

    /* --- Step 1: Desired yaw rate (dynamic, understeer-gradient aware) ---
     *
     * The classic kinematic estimate  r = v*tan(delta)/L  is the ZERO-slip yaw
     * rate.  A real car running a body-slip angle never reaches it, so it
     * over-demands yaw mid-corner.  The steady-state single-track model bends
     * the reference down with a v^2 understeer term so it tracks the achievable
     * yaw rate:  r = v * tan(delta) / (L + K_us * v^2).  K_us = 0 recovers the
     * kinematic estimate. */
    float desired_yaw_rate = 0.0f;
    if (sensors->velocity > 0.5f) {
        float v = sensors->velocity;
        desired_yaw_rate = v * tanf(sensors->steering_angle)
                           / (TV_WHEELBASE_M + TV_K_US * v * v);
    }

    /* --- Step 1b: Fuse the IMU yaw rate with a wheel-speed estimate ---
     *
     *   r_wheels = (v_right - v_left) / track,   v = wheel_speed * radius
     *
     * averaged front/rear, then blended with the IMU.  The wheel channel
     * corroborates the gyro (catching IMU bias/drift) but is down-weighted
     * because it loses validity once a tyre slips and no longer rolls cleanly. */
    float v_fl = sensors->wheel_speed[WHEEL_FL] * TV_WHEEL_RADIUS_M;
    float v_fr = sensors->wheel_speed[WHEEL_FR] * TV_WHEEL_RADIUS_M;
    float v_rl = sensors->wheel_speed[WHEEL_RL] * TV_WHEEL_RADIUS_M;
    float v_rr = sensors->wheel_speed[WHEEL_RR] * TV_WHEEL_RADIUS_M;

    float r_front  = (v_fr - v_fl) / TV_TRACK_WIDTH_M;
    float r_rear   = (v_rr - v_rl) / TV_TRACK_WIDTH_M;
    float r_wheels = 0.5f * (r_front + r_rear);

    float yaw_rate_est = (1.0f - TV_WHEEL_YAW_TRUST) * sensors->yaw_rate
                         +        TV_WHEEL_YAW_TRUST  * r_wheels;

    /* --- Step 2: Yaw rate error, with deadband ---
     *
     * The deadband holds the differential at zero until there is a real error
     * to correct, so the bias does not chatter on sensor noise plus the
     * steady-state bias of the kinematic reference. */
    float yaw_error = desired_yaw_rate - yaw_rate_est;
    if (yaw_error >  TV_YAW_DEADBAND) yaw_error -= TV_YAW_DEADBAND;
    else if (yaw_error < -TV_YAW_DEADBAND) yaw_error += TV_YAW_DEADBAND;
    else yaw_error = 0.0f;

    /* Seed the derivative memory on the very first call so the first tick does
     * not see a huge (error - 0) spike. */
    if (!g_primed) { g_prev_error = yaw_error; g_primed = 1; }

    /* --- Step 3: Speed-scheduled master gain ---
     *
     * The same torque differential produces a larger yaw response, relative to
     * the available tyre force, at higher speed.  Scale Kp inversely with speed
     * so the moment authority feels consistent, capped near standstill. */
    float effective_kp = kp_yaw;
    if (sensors->velocity > 2.0f) {
        effective_kp = kp_yaw * (TV_SPEED_REF_MS / sensors->velocity);
        if (effective_kp > kp_yaw * 3.0f) effective_kp = kp_yaw * 3.0f;
    }

    /* --- Step 4: Feedforward term ---
     *
     * Pre-load the differential from the cornering demand the instant the
     * steering moves, so feedback only has to trim.  Scales with desired yaw
     * rate and speed (the heavier the corner, the more moment it needs). */
    float ff = 0.0f;
    if (kp_yaw > 0.0f && sensors->velocity > 2.0f)
        ff = TV_KFF * desired_yaw_rate * sensors->velocity;

    /* --- Step 5: PID feedback ---
     *
     * P trims the residual; I erases the standing understeer; D damps turn-in.
     * Gain = 0 disables the whole feedback+FF path (used as the TV-off baseline
     * in tests), so honour it exactly. */
    float p_term = effective_kp * yaw_error;

    float d_error = (yaw_error - g_prev_error) / TV_DT_S;
    float d_term  = (TV_KD_FRAC * effective_kp) * d_error;
    g_prev_error  = yaw_error;

    /* Provisional integral update (may be rolled back below by anti-windup). */
    float ki = TV_KI_FRAC * effective_kp;
    float integ_next = g_integ + yaw_error * TV_DT_S;
    /* Hard cap the integral *contribution* to bias. */
    float i_contrib = ki * integ_next;
    if (i_contrib >  TV_I_MAX_NM) { integ_next =  TV_I_MAX_NM / (ki > 1e-6f ? ki : 1e-6f); }
    if (i_contrib < -TV_I_MAX_NM) { integ_next = -TV_I_MAX_NM / (ki > 1e-6f ? ki : 1e-6f); }
    float i_term = ki * integ_next;

    float bias = 0.0f;
    if (kp_yaw > 0.0f)
        bias = ff + p_term + i_term + d_term;

    /* Clamp total bias to half the motor peak.  Allows the inner wheel into
     * regen while the outer drives, for a stronger moment than drive-only. */
    float max_bias = MAX_MOTOR_TORQUE_NM * 0.5f;
    int   bias_clamped = 0;
    if (bias >  max_bias) { bias =  max_bias; bias_clamped = 1; }
    if (bias < -max_bias) { bias = -max_bias; bias_clamped = 1; }

    /* Anti-windup: only commit the integrator if the bias is NOT clamped, OR
     * the error is driving the bias back out of saturation.  Otherwise hold the
     * integrator so it cannot wind up against the rail. */
    float integ_prev = g_integ;            /* for hardware-saturation rollback */
    if (kp_yaw <= 0.0f) {
        g_integ = 0.0f;                     /* TV off: keep state clean */
    } else if (!bias_clamped ||
               (bias >= max_bias && yaw_error < 0.0f) ||
               (bias <= -max_bias && yaw_error > 0.0f)) {
        g_integ = integ_next;
    }
    /* else: leave g_integ unchanged (frozen). */

    /* --- Step 6: Base torque per wheel (before bias) ---
     * driver_torque is the total MOTOR torque demand; the four motor outputs are
     * also motor torque (the vehicle model applies the gear ratio itself).  Just
     * split the demand four ways; do NOT pre-multiply by the gear ratio. */
    float base_per_wheel = driver_torque * 0.25f;

    /* --- Step 7: Split the differential front/rear, then apply per side ---
     *
     * Sign convention: positive steering = turning left; the RIGHT wheels are
     * then the outer wheels and should get MORE torque, so positive bias adds
     * to the right and subtracts from the left.  The rear axle carries the
     * larger share of the differential (more spare longitudinal grip there).
     *
     * `bias` is the per-axle differential magnitude.  The shares redistribute it
     * between the two axles around a mean of 1.0 (so they are weighted by 2x the
     * share): TV_REAR_SHARE = 0.5 gives the symmetric full-bias-on-both-axles
     * behaviour, while a rearward share moves authority to the rear axle WITHOUT
     * reducing the total yaw moment the two axles produce together. */
    float bias_front = bias * (2.0f * TV_FRONT_SHARE);
    float bias_rear  = bias * (2.0f * TV_REAR_SHARE);

    /* --- Step 8: Clamp to motor limits, PRESERVING the differential ---
     *
     * A naive per-wheel clamp silently destroys the yaw moment when the outer
     * wheel saturates.  clamp_axle_preserving() transfers the clipped amount
     * onto the inner wheel (which has regen headroom) so the differential — and
     * the moment — is held as far as the motor envelope allows. */
    int sat_f = clamp_axle_preserving(base_per_wheel - bias_front * 0.5f,
                                      base_per_wheel + bias_front * 0.5f,
                                      &out->fl, &out->fr);
    int sat_r = clamp_axle_preserving(base_per_wheel - bias_rear * 0.5f,
                                      base_per_wheel + bias_rear * 0.5f,
                                      &out->rl, &out->rr);

    /* Hardware-saturation anti-windup: if a motor physically ran out of envelope
     * (not just the soft bias clamp) while the error was still driving the bias
     * further in the same direction, roll the integrator back to its pre-update
     * value.  Integrating against torque the hardware cannot deliver would only
     * wind up and overshoot once grip/headroom returns. */
    if ((sat_f || sat_r) && kp_yaw > 0.0f) {
        if ((bias >= 0.0f && yaw_error > 0.0f) ||
            (bias <  0.0f && yaw_error < 0.0f))
            g_integ = integ_prev;
    }
}
