#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "track.h"

/*
 * path_planning.h
 *
 * Builds the racing-line waypoints from the raw cone positions stored in
 * a Track struct.  Runs once at startup; output is written directly into
 * track->points[] / track->count so the rest of the system is unchanged.
 *
 * Three-stage algorithm
 * ---------------------
 *
 * Stage 1 — Gate extraction (nearest-right-cone pairing)
 *   For each left cone (in the order already stored, i.e. along the track),
 *   find the nearest right cone and form a gate.  Gates wider than 10 m are
 *   rejected.  Anchoring on every left cone in cone order avoids the mutual-
 *   nearest-neighbour failure at hairpins where many outside cones share one
 *   inside apex, and it naturally sorts the gates without a separate chain sort.
 *
 * Stage 2 — Centreline resampling
 *   Gate midpoints form a raw centreline with uneven spacing.  The centreline
 *   is resampled to uniform 2.5 m arc-length spacing so the optimiser sees
 *   evenly spaced control points (uneven spacing produced a start-straight spike
 *   when two adjacent cones mapped to the same opposite cone).  Each resampled
 *   point also carries the interpolated corridor half-width and a unit track
 *   normal computed from neighbouring points.
 *
 * Stage 3 — Minimum-curvature racing line (global bending-energy minimisation)
 *   Each resampled point has one degree of freedom: a lateral offset along its
 *   track normal.  We minimise E = sum_i ||P_{i-1} - 2 P_i + P_{i+1}||^2 by
 *   sweeping the [1,-4,6,-4,1] Gauss-Seidel update over all offsets, clamped to
 *   the corridor (RACING_MARGIN from each boundary).  Wider arcs through corners
 *   raise the speed limit computed by motion_control's curvature-based planner.
 */

void path_plan(Track *track);

#endif /* PATH_PLANNING_H */
