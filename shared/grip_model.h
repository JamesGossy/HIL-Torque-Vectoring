#ifndef GRIP_MODEL_H
#define GRIP_MODEL_H

#include "vehicle_config.h"

/*
 * Single source of truth for the car's lateral grip. The speed planner,
 * friction-circle budget, throttle traction cut, and racing line all read the
 * same physically-derived peak from here. Derived from vehicle_config.h, no
 * tuning. GRIP_USE in tunables.c is the only knob.
 */

#define PEAK_LAT_FLAT (MU_TYRE * 9.81f) // flat zero-downforce tyre limit, m/s^2

// Speed-dependent physical peak lateral grip, m/s^2.
static inline float peak_lat(float v)
{
    return lateral_grip_accel(PEAK_LAT_FLAT, v);
}

#endif /* GRIP_MODEL_H */
