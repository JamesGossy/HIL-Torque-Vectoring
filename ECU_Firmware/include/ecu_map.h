#ifndef ECU_MAP_H
#define ECU_MAP_H

/*
 * The ECU's own map of the track: the cone landmarks it has placed and the
 * waypoint line it plans through them. This is the ECU-side replacement for the
 * HIL Track struct, so the autonomy stack never reaches across the HIL boundary.
 * The waypoint/progress fields mirror Track on purpose: the moved planner and
 * motion controller operate on them unchanged.
 */

#include "../../shared/vehicle_config.h"

/* ---- sizing limits ---- */

#define ECU_MAX_WAYPOINTS 200
#define ECU_MAX_CONES 150

/* ---- types ---- */

typedef struct {
    float x;
    float y;
} MapPoint;

typedef struct {
    MapPoint points[ECU_MAX_WAYPOINTS];
    int count;         // waypoints used
    int current_index; // waypoint the car is heading for
    int lap_count;     // full laps completed

    MapPoint left_cones[ECU_MAX_CONES];
    int left_count;
    MapPoint right_cones[ECU_MAX_CONES];
    int right_count;
} EcuMap;

#endif /* ECU_MAP_H */
