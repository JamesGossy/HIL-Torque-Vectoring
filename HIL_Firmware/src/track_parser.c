#include "../include/track_parser.h"
#include "../include/path_planning.h"
#include "../include/track_data.h"   /* generated from the track YAML by tools/gen_tracks.py */

#include <stdlib.h>
#include <string.h>

/*
 * track_parser.c - select a cone layout and build the racing line from it.
 *
 * The cone arrays and the TRACK_DATA[] table are generated into track_data.h
 * from the track YAML (the source of truth) by tools/gen_tracks.py, which the
 * Makefile runs before compiling. This file picks one layout by name - the
 * TRACK environment variable, defaulting to "fsg2024" - copies its boundary
 * cones into the Track, and calls path_plan() to build the waypoints.
 *
 * To add or edit a track, change the YAML and rebuild; nothing here needs to
 * change. Every sim entry point goes through track_init(), and the visualiser
 * draws whatever cones the program prints.
 */

#define DEFAULT_TRACK "fsg2024"

/* Select the layout named by the TRACK environment variable, falling back to
 * DEFAULT_TRACK if TRACK is unset or names an unknown layout. */
static const TrackLayout *select_layout(void)
{
    const char *want = getenv("TRACK");
    if (want && *want) {
        for (int i = 0; i < TRACK_DATA_COUNT; i++) {
            if (strcmp(want, TRACK_DATA[i].name) == 0) return &TRACK_DATA[i];
        }
    }
    for (int i = 0; i < TRACK_DATA_COUNT; i++) {
        if (strcmp(DEFAULT_TRACK, TRACK_DATA[i].name) == 0) return &TRACK_DATA[i];
    }
    return &TRACK_DATA[0];
}


/* ------------------------------------------------------------------ */
/* Track init                                                           */
/* ------------------------------------------------------------------ */

void track_init(Track *track)
{
    const TrackLayout *layout = select_layout();
    int i;

    track->count         = 0;
    track->current_index = 0;
    track->lap_count     = 0;

    /* Store raw cone positions for the visualiser and path planner */
    for (i = 0; i < layout->left_count && i < MAX_CONES; i++) {
        track->left_cones[i].x = layout->left[i][0];
        track->left_cones[i].y = layout->left[i][1];
    }
    track->left_count = (layout->left_count < MAX_CONES) ? layout->left_count : MAX_CONES;

    for (i = 0; i < layout->right_count && i < MAX_CONES; i++) {
        track->right_cones[i].x = layout->right[i][0];
        track->right_cones[i].y = layout->right[i][1];
    }
    track->right_count = (layout->right_count < MAX_CONES) ? layout->right_count : MAX_CONES;

    /* Build racing-line waypoints from the cone positions */
    path_plan(track);
}


/* ------------------------------------------------------------------ */
/* Waypoint advance                                                     */
/* ------------------------------------------------------------------ */

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

        float dist_cur_sq  = dx_cur  * dx_cur  + dy_cur  * dy_cur;
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
