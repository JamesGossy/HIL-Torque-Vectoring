#ifndef TRACK_PARSER_H
#define TRACK_PARSER_H

/*
 * track_parser.h
 * Loads a cone layout and builds the racing line. Layouts come from YAML files in
 * tracks/, generated into track_data.h by gen_tracks.py. Selected at runtime by
 * the TRACK env var (default fsg2024).
 */

/* ---- types ---- */

// A single point on the track centreline, in world metres.
typedef struct {
    float x;
    float y;
} TrackPoint;


/* ---- sizing limits ---- */

#define MAX_WAYPOINTS 200
#define MAX_CONES 150
#define WAYPOINT_CAPTURE_RADIUS_M 2.0f // distance to advance to next waypoint, metres

/* ---- track structures ---- */

// One named cone layout, with cone arrays generated into track_data.h.
typedef struct {
    const char *name;
    const float (*left)[2];
    int left_count;
    const float (*right)[2];
    int right_count;
} TrackLayout;


// The track data, filled in by track_init().
typedef struct {
    TrackPoint points[MAX_WAYPOINTS];
    int count;         // waypoints used
    int current_index; // waypoint the car is heading for
    int lap_count;     // full laps completed

    TrackPoint left_cones[MAX_CONES]; // boundary cones for visualisation
    int left_count;
    TrackPoint right_cones[MAX_CONES];
    int right_count;
} Track;

/* ---- track api ---- */

// Build the track and racing line, called once at startup.
void track_init(Track *track);

// Advance the current waypoint if reached and update the lap counter.
void track_update(Track *track, float car_x, float car_y);

#endif /* TRACK_PARSER_H */
