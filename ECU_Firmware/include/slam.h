#ifndef SLAM_H
#define SLAM_H

/*
 * EKF-SLAM. State is [px, py, theta] followed by 2 entries per cone landmark.
 * Predict integrates wheel-speed/yaw-rate odometry (from SensorData, never
 * ground truth); update fuses each range/bearing cone observation with
 * nearest-landmark, colour-gated Mahalanobis association. Full dense
 * covariance, fixed cap. See ecu_map.h for the exported map type.
 */

#include "ecu_map.h"
#include "../../shared/tv_interface.h"

#define SLAM_POSE_DIM      3
#define SLAM_MAX_LANDMARKS (2 * ECU_MAX_CONES)
#define SLAM_MAX_DIM       (SLAM_POSE_DIM + 2 * SLAM_MAX_LANDMARKS)

typedef struct {
    int color;      // CONE_COLOR_LEFT / RIGHT
    int seen_count; // observations associated to this landmark
    int slot;       // index of lx in mu (ly = slot + 1)
} Landmark;

typedef struct {
    float mu[SLAM_MAX_DIM];                  // [px,py,theta, lx0,ly0, lx1,ly1, ...]
    float P[SLAM_MAX_DIM * SLAM_MAX_DIM];    // full covariance, row-major
    int dim;                                 // 3 + 2 * n_land
    int n_land;
    Landmark land[SLAM_MAX_LANDMARKS];

    int start_land_ids[8]; // first landmarks seen, for loop closure
    int n_start_land;
    int loop_closed;
    float start_x, start_y; // seeded start pose, for the loop-closure distance test
    int moved_away;         // has the car left the start neighbourhood at least once
} SlamState;

void slam_init(SlamState *s, float start_x, float start_y, float start_heading);
void slam_predict(SlamState *s, const SensorData *sensors, float dt);
void slam_update(SlamState *s, const ConeScan *scan);
int slam_loop_closed(const SlamState *s);
void slam_get_pose(const SlamState *s, float *x, float *y, float *heading);

// Export confident landmarks into an EcuMap's cone arrays (clears its waypoints).
void slam_export_cones(const SlamState *s, EcuMap *out);

// Same, but with an explicit minimum sighting count (phase-1 line uses a lower bar).
void slam_export_cones_min(const SlamState *s, EcuMap *out, int min_sightings);

#endif /* SLAM_H */
