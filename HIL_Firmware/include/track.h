#ifndef TRACK_H
#define TRACK_H

/*
 * track.h
 *
 * The track is a figure-8 shape made of two circular loops joined in the middle.
 * It is stored as a list of (x, y) waypoints along the centreline. The car
 * follows these waypoints one by one, looping forever.
 *
 * The visualiser uses the waypoints to draw the track outline on screen.
 * The autopilot uses the waypoints to decide which way to steer.
 */


/* A single point on the track centreline, in world metres. */
typedef struct {
    float x;
    float y;
} TrackPoint;


/* The maximum number of waypoints we support. The figure-8 uses far fewer. */
#define MAX_WAYPOINTS 200

/* How close the car needs to get to a waypoint before we advance to the next one, metres. */
#define WAYPOINT_CAPTURE_RADIUS_M  4.0f


/* The track data. Filled in by track_init(). */
typedef struct {
    TrackPoint points[MAX_WAYPOINTS];
    int        count;          /* How many waypoints are actually used */
    int        current_index;  /* Which waypoint the car is currently heading for */
    int        lap_count;      /* How many full laps the car has completed */
} Track;


/* Build the figure-8 track. Call this once at startup. */
void track_init(Track *track);

/*
 * Check if the car has reached the current waypoint. If it has, advance to
 * the next one and update the lap counter.
 *
 * car_x, car_y -- the car's current world position
 */
void track_update(Track *track, float car_x, float car_y);

/*
 * Return the waypoint the car should be aiming at right now.
 * This is used by the autopilot to compute the steering angle.
 */
TrackPoint track_get_target(const Track *track);

/*
 * The world bounding box of the track -- used by the visualiser to set up the
 * coordinate scale. All waypoints fit inside this box.
 */
void track_get_bounds(const Track *track,
                      float *min_x, float *max_x,
                      float *min_y, float *max_y);

#endif /* TRACK_H */
