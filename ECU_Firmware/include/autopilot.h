#ifndef AUTOPILOT_H
#define AUTOPILOT_H

/*
 * Top-level autonomous ECU. Owns SLAM, the online planner, the motion
 * controller and torque vectoring. Each tick it consumes a SensorData (odometry
 * + cone observations) and produces a DriveCommand (steering + wheel torques).
 * It never sees ground truth: the HIL applies the command to the plant.
 */

#include "../../shared/tv_interface.h"
#include "slam.h"

// Reset all internal state and seed the SLAM pose at the known start.
void autopilot_init(float start_x, float start_y, float start_heading);

// The internal SLAM state, for diagnostics (map RMSE) on the HIL side.
const SlamState *autopilot_slam(void);

// One control tick. Reads sensors (incl. cone scan), writes the drive command.
void autopilot_update(const SensorData *sensors, float dt, DriveCommand *cmd);

// Read the current SLAM pose estimate (for diagnostics/scoring on the HIL side).
void autopilot_get_pose(float *x, float *y, float *heading);

// Number of mapped landmarks (diagnostics).
int autopilot_landmark_count(void);

#endif /* AUTOPILOT_H */
