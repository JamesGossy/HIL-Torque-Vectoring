#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "ecu_map.h"

/* Builds the racing-line waypoints from cone positions in an EcuMap. */

// Builds the racing line and writes into map->points[] / map->count.
void path_plan(EcuMap *map);

#endif /* PATH_PLANNING_H */
