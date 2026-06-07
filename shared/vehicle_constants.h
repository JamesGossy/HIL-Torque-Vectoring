#ifndef VEHICLE_CONSTANTS_H
#define VEHICLE_CONSTANTS_H

/*
 * shared/vehicle_constants.h — M25 physical constants shared across the HIL/ECU boundary
 *
 * Both the HIL simulation (vehicle_config.h) and the ECU firmware
 * (torque_vectoring.c) need these geometry values.  Keeping them in one place
 * ensures they can never silently drift apart — which would be exactly the bug
 * class HIL is designed to catch.
 *
 * Include this file from both sides instead of defining the values twice.
 * The ECU is allowed to include shared/ headers (only tv_interface.h and this
 * file); it must not include anything under HIL_Firmware/.
 */

/* ---- M25 geometry ---- */
#define SHARED_WHEELBASE_M      1.55f   /* lf + lr, metres            */
#define SHARED_TRACK_WIDTH_M    1.30f   /* front = rear track width   */
#define SHARED_WHEEL_RADIUS_M   0.254f  /* nominal tyre rolling radius */
#define SHARED_GEAR_RATIO       15.47f  /* motor-to-wheel gear ratio  */

#endif /* VEHICLE_CONSTANTS_H */
