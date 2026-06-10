#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "track_parser.h"

/* Builds the racing-line waypoints from cone positions in a Track. */

// Builds the racing line and writes into track->points[] / track->count.
void path_plan(Track *track);

#endif /* PATH_PLANNING_H */
