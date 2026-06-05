#ifndef TV_INTERFACE_H
#define TV_INTERFACE_H

/*
 * tv_interface.h
 *
 * This file is the contract between the HIL simulation and the ECU firmware.
 *
 * In a real HIL system, these structs would be packed and sent over a CAN bus
 * or SPI link. Here, both sides are compiled into one binary, so we just pass
 * pointers. But keeping the structs in a shared file means you can see exactly
 * what crosses the HIL/ECU boundary -- just like in the real system.
 */


/* What the ECU can see from its sensors.
 * The HIL fills this in from the vehicle model each tick and hands it to
 * the torque vectoring algorithm. The ECU never touches the full VehicleState
 * -- it only gets what a real sensor suite would provide. */
typedef struct {
    float yaw_rate;        /* rad/s  -- from the IMU gyroscope */
    float velocity;        /* m/s    -- average of all four wheel speeds */
    float steering_angle;  /* rad    -- from the steering angle sensor */
    float wheel_speed[4];  /* rad/s  -- FL, FR, RL, RR from wheel encoders */
    float driver_torque;   /* Nm     -- total torque requested by the driver (throttle pedal) */
} SensorData;


/* The torque the ECU wants to send to each motor.
 * The TV algorithm fills this in and hands it back to the HIL,
 * which applies it to the vehicle model on the next tick. */
typedef struct {
    float fl;  /* Front-left motor,  Nm */
    float fr;  /* Front-right motor, Nm */
    float rl;  /* Rear-left motor,   Nm */
    float rr;  /* Rear-right motor,  Nm */
} WheelTorques;


/* Wheel index constants -- use these instead of magic numbers 0-3. */
#define WHEEL_FL 0
#define WHEEL_FR 1
#define WHEEL_RL 2
#define WHEEL_RR 3

#endif /* TV_INTERFACE_H */
