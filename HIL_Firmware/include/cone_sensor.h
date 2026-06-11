#ifndef CONE_SENSOR_H
#define CONE_SENSOR_H

/*
 * Simulated cone detector. The 2D sim has no camera, so this synthesises noisy
 * range/bearing observations of the track cones from the ground-truth pose,
 * inside a field of view and range. It is HIL-side (it reads ground truth); the
 * ECU only ever sees the resulting ConeScan inside SensorData.
 */

#include "track_parser.h"
#include "../../shared/tv_interface.h"

// Fill scan with noisy observations of cones visible from the ground-truth pose.
void cone_sensor_scan(
    const Track *track, float gt_x, float gt_y, float gt_heading, ConeScan *scan);

// Reseed the noise RNG so a run is reproducible.
void cone_sensor_reset(void);

#endif /* CONE_SENSOR_H */
