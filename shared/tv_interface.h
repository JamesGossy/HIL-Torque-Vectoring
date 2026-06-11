#ifndef TV_INTERFACE_H
#define TV_INTERFACE_H

/* The data contract between the HIL simulation and the ECU firmware. Both sides
 * compile into one binary, but keeping these structs shared shows exactly what
 * crosses the HIL/ECU boundary. */

/* ---- cone perception ---- */

#define CONE_COLOR_LEFT  0 /* blue */
#define CONE_COLOR_RIGHT 1 /* yellow */
#define MAX_OBS_PER_TICK 32

/* One detected cone, body frame: bearing +ve to the left of vehicle heading. */
typedef struct {
    float range;   /* m */
    float bearing; /* rad, CCW+ from heading */
    int color;     /* CONE_COLOR_LEFT / CONE_COLOR_RIGHT */
} ConeObservation;

/* The cones seen this tick. */
typedef struct {
    ConeObservation obs[MAX_OBS_PER_TICK];
    int count;
} ConeScan;

/* What the ECU can see from its sensors. */
typedef struct {
    float yaw_rate;       /* rad/s, from the IMU gyroscope */
    float velocity;       /* m/s, average of all four wheel speeds */
    float steering_angle; /* rad, from the steering angle sensor */
    float wheel_speed[4]; /* rad/s, FL, FR, RL, RR from wheel encoders */
    float driver_torque;  /* Nm, driver demand (legacy non-autonomous path only) */
    ConeScan scan;        /* cones detected this tick, for SLAM */
} SensorData;

/* The torque the ECU wants to send to each motor. */
typedef struct {
    float fl; /* front-left, Nm */
    float fr; /* front-right, Nm */
    float rl; /* rear-left, Nm */
    float rr; /* rear-right, Nm */
} WheelTorques;

/* The full ECU output in autonomy: steering plus per-wheel torque. */
typedef struct {
    float steering_rad;
    WheelTorques torques;
} DriveCommand;


#define WHEEL_FL 0
#define WHEEL_FR 1
#define WHEEL_RL 2
#define WHEEL_RR 3

#endif /* TV_INTERFACE_H */
