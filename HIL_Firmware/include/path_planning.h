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
 *   3. Minimum-lap-time line: seed with a minimum-curvature line, then refine
 *      the cross-track offsets by coordinate descent on the actual lap time
 *      (a forward+backward speed profile under the friction circle). Unlike a
 *      purely geometric line this couples path and speed, so it produces late
 *      apexes - sacrificing apex speed to straighten a corner exit onto a
 *      straight. See path_planning.c for the full method.
 */

void path_plan(Track *track);

#endif /* PATH_PLANNING_H */
