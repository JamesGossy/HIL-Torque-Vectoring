#include "../include/torque_vectoring.h"
#include "../../shared/vehicle_config.h"
#include <math.h>

/*
 * torque_vectoring.c
 *
 * The ECU torque vectoring algorithm.
 *
 * Plain English explanation:
 *
 *   When a car goes around a corner, the outside wheels need to travel a longer
 *   arc than the inside wheels. If all four wheels receive the same torque, the
 *   car tends to push straight (understeer). If we give MORE torque to the
 *   outside wheels, they pull the car around the corner better. This is torque
 *   vectoring.
 *
 *   We use a yaw-rate error to decide how much to shift torque left or right:
 *
 *     Step 1: Calculate the yaw rate the car SHOULD have.
 *             At any given speed and steering angle, a car on the correct path
 *             should be rotating at a predictable rate. This comes from simple
 *             geometry of a bicycle model:
 *
 *               desired_yaw_rate = speed * tan(steering_angle) / wheelbase
 *
 *     Step 2: Measure the yaw rate the car ACTUALLY has (from the IMU sensor).
 *
 *     Step 3: Compute the error.
 *               error = desired - actual
 *             If positive: car is not rotating enough (understeering). Give more
 *             torque to the outer wheels to pull it around.
 *             If negative: car is rotating too much (oversteering). Give less
 *             torque to the outer wheels.
 *
 *     Step 4: Compute a torque bias using proportional gain.
 *               bias = Kp * error
 *
 *     Step 5: Split the total driver torque across the four wheels.
 *             - 50% goes to the front axle, 50% to the rear (fixed split).
 *             - Within each axle, the bias shifts torque left or right.
 *               If turning left (negative steering), the right wheels are outer.
 *               Right wheels get (base + bias/2), left wheels get (base - bias/2).
 *
 *     Step 6: Clamp each wheel torque to the motor's physical limits.
 *
 *   That's the whole algorithm. It is a simple P-controller on yaw rate.
 *   Real production TV systems use more complex control (PID, feedforward,
 *   yaw moment maps, tyre models) but this captures the essential idea.
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


/* Clamp one axle's left/right torques to the motor envelope while PRESERVING
 * the commanded differential (left - right), which is what carries the yaw
 * moment.  If one side exceeds the peak, the excess is shifted onto the other
 * side (into regen if needed) before the hard clamp, so a saturated outer wheel
 * no longer silently collapses the differential. */
static void clamp_axle_preserving(float left, float right,
                                  float *left_out, float *right_out)
{
    if (left > MAX_MOTOR_TORQUE_NM) {
        right -= (left - MAX_MOTOR_TORQUE_NM);
        left   = MAX_MOTOR_TORQUE_NM;
    } else if (right > MAX_MOTOR_TORQUE_NM) {
        left -= (right - MAX_MOTOR_TORQUE_NM);
        right = MAX_MOTOR_TORQUE_NM;
    }
    if (left  < MIN_MOTOR_TORQUE_NM) left  = MIN_MOTOR_TORQUE_NM;
    if (left  > MAX_MOTOR_TORQUE_NM) left  = MAX_MOTOR_TORQUE_NM;
    if (right < MIN_MOTOR_TORQUE_NM) right = MIN_MOTOR_TORQUE_NM;
    if (right > MAX_MOTOR_TORQUE_NM) right = MAX_MOTOR_TORQUE_NM;
    *left_out  = left;
    *right_out = right;
}


void torque_vectoring_update(const SensorData *sensors,
                             float             driver_torque,
                             float             kp_yaw,
                             WheelTorques     *out)
{
    /* --- Step 1: Desired yaw rate (dynamic, understeer-gradient aware) ---
     *
     * The classic kinematic estimate  r = v*tan(delta)/L  is the ZERO-slip yaw
     * rate.  A real car running a body-slip angle never reaches it, so at corner
     * speed it over-demands yaw (measured here ~1.85 rad/s too high mid-corner),
     * and the controller chases a target the tyres cannot deliver.
     *
     * The steady-state single-track model adds an understeer term:
     *   r = v * delta / (L + K_us * v^2)
     * The v^2 term bends the reference down as the car loads up laterally, so it
     * tracks the achievable yaw rate.  K_us = 0 recovers the kinematic estimate.
     * This is a pure mapping (no controller state) — see the note on why no
     * integral term was added. */
    float desired_yaw_rate = 0.0f;
    if (sensors->velocity > 0.5f) {
        float v = sensors->velocity;
        desired_yaw_rate = v * tanf(sensors->steering_angle)
                           / (TV_WHEELBASE_M + TV_K_US * v * v);
    }

    /* --- Step 1b: Fuse the IMU yaw rate with a wheel-speed estimate ---
     *
     * On a cornering car the outer wheels travel faster than the inner ones.
     * The yaw rate follows directly from the left/right ground-speed split:
     *
     *   r_wheels = (v_right - v_left) / track
     *
     * where v = wheel_speed (rad/s) * wheel_radius.  We average the front and
     * rear axle estimates, then blend with the IMU.  This corroborates the
     * gyro (catching IMU bias/drift) but is down-weighted because it loses
     * validity once a tyre slips or scrubs and no longer rolls cleanly. */
    float v_fl = sensors->wheel_speed[WHEEL_FL] * TV_WHEEL_RADIUS_M;
    float v_fr = sensors->wheel_speed[WHEEL_FR] * TV_WHEEL_RADIUS_M;
    float v_rl = sensors->wheel_speed[WHEEL_RL] * TV_WHEEL_RADIUS_M;
    float v_rr = sensors->wheel_speed[WHEEL_RR] * TV_WHEEL_RADIUS_M;

    float r_front  = (v_fr - v_fl) / TV_TRACK_WIDTH_M;
    float r_rear   = (v_rr - v_rl) / TV_TRACK_WIDTH_M;
    float r_wheels = 0.5f * (r_front + r_rear);

    float yaw_rate_est = (1.0f - TV_WHEEL_YAW_TRUST) * sensors->yaw_rate
                         +        TV_WHEEL_YAW_TRUST  * r_wheels;

    /* --- Step 2: Yaw rate error --- */
    float yaw_error = desired_yaw_rate - yaw_rate_est;

    /* Deadband: ignore yaw errors smaller than TV_YAW_DEADBAND.  The desired
     * yaw rate is only an approximation (kinematic bicycle model) and the
     * estimate carries sensor noise, so a small residual error is always
     * present.  Reacting to it makes the bias chatter about zero; the deadband
     * holds the differential at zero until there is a real error to correct. */
    if (yaw_error >  TV_YAW_DEADBAND) yaw_error -= TV_YAW_DEADBAND;
    else if (yaw_error < -TV_YAW_DEADBAND) yaw_error += TV_YAW_DEADBAND;
    else yaw_error = 0.0f;

    /* --- Step 3: Torque bias (speed-dependent gain) ---
     *
     * At higher speed, the same left/right torque differential produces a
     * larger body-slip change relative to available tyre force.  We scale Kp
     * inversely with speed so the yaw moment response feels consistent across
     * the whole speed range.  The cap at 3×kp prevents over-aggressive
     * response at near-zero speed (e.g. launch). */
    float effective_kp = kp_yaw;
    if (sensors->velocity > 2.0f) {
        effective_kp = kp_yaw * (TV_SPEED_REF_MS / sensors->velocity);
        if (effective_kp > kp_yaw * 3.0f) effective_kp = kp_yaw * 3.0f;
    }
    float bias = effective_kp * yaw_error;

    /* Clamp bias to half the motor's peak torque capacity.
     * This allows the inner wheel to go into regen while the outer drives,
     * creating a stronger yaw moment than a drive-only differential. */
    float max_bias = MAX_MOTOR_TORQUE_NM * 0.5f;
    if (bias >  max_bias) bias =  max_bias;
    if (bias < -max_bias) bias = -max_bias;

    /* --- Step 4: Base torque per wheel (before bias) ---
     * driver_torque is the total MOTOR torque demand (Nm).  The four motor
     * outputs (WheelTorques) are also motor torque -- the vehicle model applies
     * the gear ratio itself when turning motor torque into wheel force.  So we
     * just split the demand four ways here; we must NOT pre-multiply by the gear
     * ratio (doing so double-counts it and saturates every wheel at the clamp,
     * which is what made all four wheels read the same value). */
    float base_per_wheel = driver_torque * 0.25f;

    /* --- Step 5: Apply left/right bias ---
     *
     * Steering angle sign convention:
     *   Positive steering = turning left.
     *   When turning left, the RIGHT wheels are on the outside of the corner.
     *   We want to give MORE torque to the outside (right) wheels.
     *   A positive yaw rate error means we need MORE rotation (left turn needs
     *   more yaw rate), so we add bias to the right wheels and subtract from left.
     *
     * Summary: outer_wheel += bias/2, inner_wheel -= bias/2
     *   Positive bias -> more torque right (helps left turn / positive yaw).
     *   Negative bias -> more torque left (helps right turn / negative yaw).
     */
    float left_add  = -bias * 0.5f;
    float right_add =  bias * 0.5f;

    /* --- Step 6: Clamp to motor limits, PRESERVING the differential ---
     *
     * The bias creates the yaw moment through the left/right torque DIFFERENCE.
     * A naive independent per-wheel clamp silently destroys that moment: when
     * the outer wheel saturates at the motor peak, (outer - inner) collapses and
     * the commanded yaw moment is quietly lost — a classic torque-vectoring bug.
     * clamp_axle_preserving() instead transfers the clipped amount onto the
     * inner wheel (which has regen headroom down to MIN_MOTOR_TORQUE_NM), so the
     * differential — and the yaw moment — is held at the commanded value as far
     * as the motor envelope allows.  The split stays 50/50 front/rear. */
    clamp_axle_preserving(base_per_wheel + left_add,  base_per_wheel + right_add,
                          &out->fl, &out->fr);
    clamp_axle_preserving(base_per_wheel + left_add,  base_per_wheel + right_add,
                          &out->rl, &out->rr);
}
