#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "track.h"

/*
 * path_planning.h
 *
 * Builds the optimal racing-line path from the raw cone positions stored in
 * a Track struct.  Algorithm runs once at startup; output is written directly
 * into track->points[] / track->count so the rest of the system is unchanged.
 *
 * Two-stage algorithm
 * -------------------
 *
 * Stage 1 — Delaunay gate detection
 *   A Bowyer-Watson Delaunay triangulation is run on all cone positions
 *   (left + right together).  Every edge that connects a left cone to a right
 *   cone defines a physical "gate" the car must pass through.  Gates are
 *   sorted along the track using the ordering already in the cone arrays.
 *
 * Stage 2 — Minimum-curvature racing line
 *   Each gate has a lateral offset alpha ∈ [MARGIN, 1-MARGIN] (0 = left cone,
 *   1 = right cone).  A coordinate-descent loop runs ternary search on each
 *   gate in turn to minimise the path curvature at that gate while the
 *   neighbouring gate positions are held fixed.  After convergence the path
 *   takes wider arcs through corners — raising the speed limit set by the
 *   curvature-based speed planner in motion_control — while staying safely
 *   inside both boundary lines.
 */

void path_plan(Track *track);

#endif /* PATH_PLANNING_H */
