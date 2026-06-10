#ifndef TV_INTERFACE_H
#define TV_INTERFACE_H

/* The data contract between the HIL simulation and the ECU firmware. Both sides
 * compile into one binary, but keeping these structs shared shows exactly what
 * crosses the HIL/ECU boundary. */

/* What the ECU can see from its sensors. */
typedef struct {
    float yaw_rate;       /* rad/s, from the IMU gyroscope */
    float velocity;       /* m/s, average of all four wheel speeds */
    float steering_angle; /* rad, from the steering angle sensor */
    float wheel_speed[4]; /* rad/s, FL, FR, RL, RR from wheel encoders */
    float driver_torque;  /* Nm, total torque requested by the driver */
} SensorData;

/* The torque the ECU wants to send to each motor. */
typedef struct {
    float fl; /* front-left, Nm */
    float fr; /* front-right, Nm */
    float rl; /* rear-left, Nm */
    float rr; /* rear-right, Nm */
} WheelTorques;


#define WHEEL_FL 0
#define WHEEL_FR 1
#define WHEEL_RL 2
#define WHEEL_RR 3

#endif /* TV_INTERFACE_H */
