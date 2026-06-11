// Simulated cone detector. Synthesises noisy range/bearing observations of the
// track cones from the ground-truth pose, within a field of view and range.
// HIL-side: it reads ground truth and writes a ConeScan the ECU consumes.

#include "../include/cone_sensor.h"
#include "../../shared/tunables.h"
#include <math.h>

static const float CS_PI = 3.14159265358979323846f;

// Deterministic RNG so eval runs are reproducible. Reseeded by cone_sensor_reset.
static unsigned s_rng = 0x9e3779b9u;

static float uniform01(void)
{
    s_rng = s_rng * 1664525u + 1013904223u; // numerical-recipes LCG
    return (float)((s_rng >> 8) & 0xffffff) / (float)0x1000000;
}

// One standard normal sample via Box-Muller.
static float gaussian(void)
{
    float u1 = uniform01();
    float u2 = uniform01();
    if (u1 < 1e-7f) u1 = 1e-7f; // avoid log(0)
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * CS_PI * u2);
}

void cone_sensor_reset(void)
{
    s_rng = 0x9e3779b9u;
}

static float wrap_pi(float a)
{
    while (a > CS_PI)
        a -= 2.0f * CS_PI;
    while (a < -CS_PI)
        a += 2.0f * CS_PI;
    return a;
}

// Test one cone against the FoV/range, and if visible append a noisy observation.
static void try_cone(ConeScan *scan, float cx, float cy, int color, float gx, float gy, float gh)
{
    float dx    = cx - gx;
    float dy    = cy - gy;
    float range = sqrtf(dx * dx + dy * dy);
    if (range > g_SENSOR_RANGE_M) return;

    float bearing = wrap_pi(atan2f(dy, dx) - gh);
    if (fabsf(bearing) > g_SENSOR_FOV_RAD) return;

    if (scan->count >= MAX_OBS_PER_TICK) return; // cap reached, drop the rest

    ConeObservation *o = &scan->obs[scan->count++];
    o->range           = range + g_SENSOR_RANGE_SIGMA_M * gaussian();
    o->bearing         = bearing + g_SENSOR_BEARING_SIGMA_RAD * gaussian();
    o->color           = color;
}

void cone_sensor_scan(const Track *track, float gx, float gy, float gh, ConeScan *scan)
{
    scan->count = 0;
    for (int i = 0; i < track->left_count; i++)
        try_cone(scan, track->left_cones[i].x, track->left_cones[i].y, CONE_COLOR_LEFT, gx, gy, gh);
    for (int i = 0; i < track->right_count; i++)
        try_cone(
            scan, track->right_cones[i].x, track->right_cones[i].y, CONE_COLOR_RIGHT, gx, gy, gh);
}
