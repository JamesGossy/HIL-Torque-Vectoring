#ifndef GRIP_MODEL_H
#define GRIP_MODEL_H

#include "vehicle_config.h"

/*
 * shared/grip_model.h
 *
 * The single source of truth for "how much lateral grip does the car have".
 *
 * Everything that used to carry its own grip budget - the speed planner's
 * corner-speed budget, the friction-circle (GG) budget, the throttle
 * traction-circle reference, and the racing line's shaping grip - now reads the
 * SAME physically-derived peak from here. The only knob left is GRIP_USE: the
 * fraction of that physical peak the car actually drives at (a margin for the
 * tracker), a runtime tunable in shared/tunables.c.
 *
 * The peak is derived straight from vehicle_config.h, no tuning:
 *
 *   peak_lat(v) = MU_TYRE * g + AERO_GRIP_COEF * v^2
 *
 * MU_TYRE*g is the flat (zero-downforce) tyre limit (~10.8 m/s^2); the aero term
 * is the speed-dependent downforce bonus already defined in vehicle_config.h.
 * This is exactly the form lateral_grip_accel() implements with base = MU_TYRE*g,
 * so the helpers there (lateral_grip_accel, apex_speed) compose with it directly.
 */

/* Flat (zero-downforce) lateral grip limit, m/s^2, straight from the tyre mu. */
#define PEAK_LAT_FLAT (MU_TYRE * 9.81f)

/* Speed-dependent physical peak lateral grip, m/s^2. */
static inline float peak_lat(float v)
{
    return lateral_grip_accel(PEAK_LAT_FLAT, v);
}

#endif /* GRIP_MODEL_H */
