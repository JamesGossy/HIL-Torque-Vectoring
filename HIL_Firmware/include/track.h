#ifndef TRACK_H
#define TRACK_H

/*
 * track.h
 *
 * The FSG 2024 endurance layout, defined by 228 measured boundary cones
 * (117 left, 111 right). track_init() stores the cones and calls path_plan()
 * to build the racing-line waypoints in track->points[] (these are the
 * optimised line, not the cone positions). The motion controller follows
 * track->points; the visualiser draws both the cones and the line.
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
#define WAYPOINT_CAPTURE_RADIUS_M  2.0f


/* The track data. Filled in by track_init(). */
typedef struct {
    TrackPoint points[MAX_WAYPOINTS];
    int        count;          /* How many centreline waypoints are used */
    int        current_index;  /* Which waypoint the car is currently heading for */
    int        lap_count;      /* How many full laps the car has completed */

    /* Boundary cone positions for visualisation */
    TrackPoint left_cones[MAX_CONES];
    int        left_count;
    TrackPoint right_cones[MAX_CONES];
    int        right_count;
} Track;


/* Build the track and racing line. Call this once at startup. */
void track_init(Track *track);

/*
 * Check if the car has reached the current waypoint. If it has, advance to the
 * next one and update the lap counter.
 */
void track_update(Track *track, float car_x, float car_y);

#endif /* TRACK_H */
