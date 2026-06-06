#include "../include/torque_vectoring.h"
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

/* Vehicle wheelbase -- must match vehicle_model.h. Kept separate here because
 * the ECU does not include the HIL headers. */
#define TV_WHEELBASE_M  2.4f


void torque_vectoring_update(const SensorData *sensors,
                             float             driver_torque,
                             float             kp_yaw,
                             WheelTorques     *out)
{
    /* --- Step 1: Desired yaw rate from the kinematic bicycle model --- */
    float desired_yaw_rate = 0.0f;
    if (sensors->velocity > 0.5f) {
        desired_yaw_rate = sensors->velocity
                           * tanf(sensors->steering_angle)
                           / TV_WHEELBASE_M;
    }

    /* --- Step 2: Yaw rate error --- */
    float yaw_error = desired_yaw_rate - sensors->yaw_rate;

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
     * Split 50/50 front/rear, then divide each axle between two wheels. */
    float base_per_wheel = driver_torque * 0.25f;  /* = total / 4 */

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

    out->fl = base_per_wheel + left_add;
    out->fr = base_per_wheel + right_add;
    out->rl = base_per_wheel + left_add;
    out->rr = base_per_wheel + right_add;

    /* --- Step 6: Clamp to motor limits --- */
    if (out->fl < MIN_MOTOR_TORQUE_NM) out->fl = MIN_MOTOR_TORQUE_NM;
    if (out->fr < MIN_MOTOR_TORQUE_NM) out->fr = MIN_MOTOR_TORQUE_NM;
    if (out->rl < MIN_MOTOR_TORQUE_NM) out->rl = MIN_MOTOR_TORQUE_NM;
    if (out->rr < MIN_MOTOR_TORQUE_NM) out->rr = MIN_MOTOR_TORQUE_NM;

    if (out->fl > MAX_MOTOR_TORQUE_NM) out->fl = MAX_MOTOR_TORQUE_NM;
    if (out->fr > MAX_MOTOR_TORQUE_NM) out->fr = MAX_MOTOR_TORQUE_NM;
    if (out->rl > MAX_MOTOR_TORQUE_NM) out->rl = MAX_MOTOR_TORQUE_NM;
    if (out->rr > MAX_MOTOR_TORQUE_NM) out->rr = MAX_MOTOR_TORQUE_NM;
}
