#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "track.h"

/*
 * path_planning.h
 *
 * Builds the racing-line waypoints from the raw cone positions in a Track.
 * Runs once at startup and writes into track->points[] / track->count.
 *
 * Three stages:
 *   1. Gate extraction: pair each left cone with its nearest right cone.
 *   2. Centreline resampling: resample gate midpoints to uniform spacing.
 *   3. Minimum-curvature line: sweep a bending-energy stencil to find the
 *      lowest-curvature line inside the cone corridor (wider arcs = more speed).
 */

void path_plan(Track *track);

#endif /* PATH_PLANNING_H */
