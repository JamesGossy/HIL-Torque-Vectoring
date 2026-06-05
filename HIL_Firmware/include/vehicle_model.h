#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "../../shared/tv_interface.h"

/*
 * vehicle_model.h
 *
 * The vehicle model is the "virtual car". It tracks the physics state of the
 * car and updates it every simulation tick based on the torque commands coming
 * from the ECU and the steering command coming from the autopilot.
 *
 * We use a single-track (bicycle) model. This treats the left and right wheels
 * on each axle as one combined wheel in the centre. It is simple but captures
 * the main behaviour: the car accelerates, steers, and develops yaw rate.
 *
 * What this model includes:
 *   - Position in the world (x, y)
 *   - Heading angle (yaw)
 *   - Longitudinal speed
 *   - Yaw rate (how fast the car is rotating)
 *   - Body slip angle (the angle between where the car points and where it moves)
 *
 * What this model does NOT include (intentionally, for simplicity):
 *   - Detailed tyre slip curves (Pacejka magic formula etc.)
 *   - Suspension and weight transfer
 *   - Aerodynamic drag and downforce
 *   - Brake torque vectoring
 */


/* The full physics state of the car.
 * The HIL owns this. The ECU never sees it directly -- it only gets
 * a SensorData snapshot (see tv_interface.h). */
typedef struct {
    float x;          /* World X position, metres (east is positive) */
    float y;          /* World Y position, metres (north is positive) */
    float heading;    /* Yaw angle, radians. 0 = pointing east. CCW is positive. */
    float velocity;   /* Longitudinal speed, m/s */
    float yaw_rate;   /* Rate of change of heading, rad/s */
    float slip_angle; /* Body slip angle (beta), radians */
    float steering;   /* Front wheel steering angle, radians (set by autopilot) */
} VehicleState;


/* Vehicle geometry and mass constants */
#define WHEELBASE_M       2.4f   /* Distance between front and rear axle, metres */
#define TRACK_WIDTH_M     1.5f   /* Distance between left and right wheels, metres */
#define MASS_KG         300.0f   /* Total car mass, kg */
#define WHEEL_RADIUS_M    0.25f  /* Driven wheel radius, metres */
#define MAX_SPEED_MS      30.0f  /* Top speed limit, m/s (~108 km/h) */
#define DRAG_COEFF       20.0f   /* Simple linear drag constant, N/(m/s) */


/* Set the vehicle to its starting position and zero all motion. */
void vehicle_model_init(VehicleState *s, float start_x, float start_y, float start_heading);

/*
 * Advance the vehicle physics by one time step.
 *
 * s       -- the current vehicle state (updated in place)
 * torques -- the wheel torques from the ECU for this tick
 * dt      -- time step in seconds (e.g. 0.01 for 100 Hz)
 */
void vehicle_model_update(VehicleState *s, const WheelTorques *torques, float dt);

#endif /* VEHICLE_MODEL_H */
