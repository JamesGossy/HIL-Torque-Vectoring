#ifndef TRACK_PARSER_H
#define TRACK_PARSER_H

/*
 * track_parser.h
 *
 * Loads one of the measured cone layouts and builds the racing line from it.
 *
 * The layouts live in the tracks/ directory as YAML (the source of truth). At
 * build time tools/gen_tracks.py turns every YAML file into track_data.h (cone
 * arrays); track_parser.c includes that, selects a layout by name - the TRACK
 * environment variable, defaulting to "fsg2024" - stores its boundary cones,
 * and calls path_plan() to build the racing-line waypoints in track->points[]
 * (the optimised line, not the cone positions). The motion controller follows
 * track->points; the visualiser draws both the cones and the line.
 *
 * Add a track by dropping another tracks/<name>.yaml in and rebuilding. Select
 * one at runtime with e.g. TRACK=fse2024 make eval.
 */


/* A single point on the track centreline, in world metres. */
typedef struct {
    float x;
    float y;
} TrackPoint;


/* The maximum number of centreline waypoints. */
#define MAX_WAYPOINTS 200

/* The maximum number of boundary cones per side. */
#define MAX_CONES 150

/* How close the car needs to get to a waypoint before we advance to the next one, metres. */
#define WAYPOINT_CAPTURE_RADIUS_M 2.0f


/* One named cone layout. The cone arrays are generated into track_data.h from
 * the track YAML; track_parser.c selects one of these by name. */
typedef struct {
    const char *name;
    const float (*left)[2];
    int left_count;
    const float (*right)[2];
    int right_count;
} TrackLayout;


/* The track data. Filled in by track_init(). */
typedef struct {
    TrackPoint points[MAX_WAYPOINTS];
    int count;         /* How many centreline waypoints are used */
    int current_index; /* Which waypoint the car is currently heading for */
    int lap_count;     /* How many full laps the car has completed */

    /* Boundary cone positions for visualisation */
    TrackPoint left_cones[MAX_CONES];
    int left_count;
    TrackPoint right_cones[MAX_CONES];
    int right_count;
} Track;


/* Build the track and racing line. Call this once at startup. */
void track_init(Track *track);

/*
 * Check if the car has reached the current waypoint. If it has, advance to the
 * next one and update the lap counter.
 */
void track_update(Track *track, float car_x, float car_y);

#endif /* TRACK_PARSER_H */
