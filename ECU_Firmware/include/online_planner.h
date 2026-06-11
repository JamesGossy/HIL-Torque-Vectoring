#ifndef ONLINE_PLANNER_H
#define ONLINE_PLANNER_H

/*
 * Two-phase online planner. Phase 1 (exploration lap): build a short reactive
 * centreline from the SLAM cone map ahead of the estimated pose and follow it
 * slowly while mapping. Phase 2 (after loop closure): run the offline racing-
 * line optimiser once on the completed map and race it. The map and pose come
 * from SLAM, so the planner never touches ground truth.
 */

#include "ecu_map.h"
#include "slam.h"

typedef struct {
    int phase2_active;  // 0 = reactive centreline, 1 = optimised racing line
    EcuMap racing_line; // the phase-2 line, planned once and re-served each tick
} OnlinePlanner;

void online_planner_init(OnlinePlanner *p);

// Rebuild the EcuMap the controller will follow this tick from the SLAM state.
// Returns a speed cap (m/s) to apply on top of the planner target (phase-1 cap),
// or a large value in phase 2 (no extra cap).
float online_planner_step(OnlinePlanner *p, const SlamState *slam, float est_x, float est_y,
    float est_heading, EcuMap *out_map);

// Pure-pursuit lookahead target on the current map->points at distance Ld ahead
// of (px,py). Writes the target point; returns 1 if a valid target was found.
int online_planner_lookahead(const EcuMap *map, float px, float py, float Ld, float *tx, float *ty);

#endif /* ONLINE_PLANNER_H */
