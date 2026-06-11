#include "../include/track_parser.h"
#include "../include/track_data.h" // generated from the track YAML by tools/gen_tracks.py
#include "../../ECU_Firmware/include/path_planning.h"
#include "../../ECU_Firmware/include/ecu_map.h"

#include <stdlib.h>
#include <string.h>

// Selects a cone layout by name and builds the racing line from it.

#define DEFAULT_TRACK "fsg2024"

// Returns the layout named by the TRACK env var, or DEFAULT_TRACK if unset or unknown.
static const TrackLayout *select_layout(void)
{
    // match the requested name first
    const char *want = getenv("TRACK");
    if (want && *want) {
        for (int i = 0; i < TRACK_DATA_COUNT; i++) {
            if (strcmp(want, TRACK_DATA[i].name) == 0) return &TRACK_DATA[i];
        }
    }
    // fall back to the default track
    for (int i = 0; i < TRACK_DATA_COUNT; i++) {
        if (strcmp(DEFAULT_TRACK, TRACK_DATA[i].name) == 0) return &TRACK_DATA[i];
    }
    return &TRACK_DATA[0];
}

// Loads the selected layout's cones into the track and builds the waypoints.
void track_init(Track *track)
{
    const TrackLayout *layout = select_layout();
    int i;

    // reset track progress
    track->count         = 0;
    track->current_index = 0;
    track->lap_count     = 0;

    // copy left cones
    for (i = 0; i < layout->left_count && i < MAX_CONES; i++) { // raw cones for visualiser and planner
        track->left_cones[i].x = layout->left[i][0];
        track->left_cones[i].y = layout->left[i][1];
    }
    track->left_count = (layout->left_count < MAX_CONES) ? layout->left_count : MAX_CONES;

    // copy right cones
    for (i = 0; i < layout->right_count && i < MAX_CONES; i++) {
        track->right_cones[i].x = layout->right[i][0];
        track->right_cones[i].y = layout->right[i][1];
    }
    track->right_count = (layout->right_count < MAX_CONES) ? layout->right_count : MAX_CONES;

    // The racing-line planner lives on the ECU now and works on an EcuMap, so build
    // the line there and copy the waypoints back into the Track for the visualiser
    // and scoring. The legacy driver gets the same line via its own track_to_ecu_map.
    static EcuMap m;
    m.left_count = track->left_count;
    for (i = 0; i < track->left_count; i++) {
        m.left_cones[i].x = track->left_cones[i].x;
        m.left_cones[i].y = track->left_cones[i].y;
    }
    m.right_count = track->right_count;
    for (i = 0; i < track->right_count; i++) {
        m.right_cones[i].x = track->right_cones[i].x;
        m.right_cones[i].y = track->right_cones[i].y;
    }

    path_plan(&m);

    track->count = (m.count < MAX_WAYPOINTS) ? m.count : MAX_WAYPOINTS;
    for (i = 0; i < track->count; i++) {
        track->points[i].x = m.points[i].x;
        track->points[i].y = m.points[i].y;
    }
}


// Advances the current waypoint index as the car passes waypoints, counting laps.
void track_update(Track *track, float car_x, float car_y)
{
    int i;
    for (i = 0; i < 5; i++) {
        int cur  = track->current_index;
        int next = (cur + 1) % track->count;

        float dx_cur  = car_x - track->points[cur].x;
        float dy_cur  = car_y - track->points[cur].y;
        float dx_next = car_x - track->points[next].x;
        float dy_next = car_y - track->points[next].y;

        float dist_cur_sq  = dx_cur * dx_cur + dy_cur * dy_cur;
        float dist_next_sq = dx_next * dx_next + dy_next * dy_next;
        float capture_sq   = WAYPOINT_CAPTURE_RADIUS_M * WAYPOINT_CAPTURE_RADIUS_M;

        if (dist_cur_sq < capture_sq || dist_next_sq < dist_cur_sq) {
            track->current_index++;
            if (track->current_index >= track->count) {
                track->current_index = 0;
                track->lap_count++;
            }
        } else {
            break;
        }
    }
}
