#include "../include/track.h"
#include <math.h>
#include <float.h>

static const float PI = 3.14159265358979323846f;

/*
 * track.c
 *
 * Defines the race track as a list of (x, y) waypoints along the centreline.
 *
 * --- How the track is built ---
 * Instead of hard-coding every waypoint, the track is described as a sequence
 * of SEGMENTS, each appended onto the end of the previous one:
 *
 *   - straight(length)      : a straight of the given length
 *   - arc(radius, degrees)  : a constant-radius bend.  Positive degrees turn
 *                             left (CCW), negative turn right (CW).
 *
 * A "turtle" walks from the current point in the current heading, laying down
 * waypoints as it goes.  This makes it easy to author tight, technical layouts:
 * just list the segments in order.  The shape is automatically closed-ish; the
 * autopilot loops from the last waypoint back to the first.
 *
 * The current layout is a TECHNICAL INFIELD: short straights linking a series
 * of hairpins and esses, demanding lots of steering input -- a good workout for
 * the torque-vectoring controller.
 *
 * The visualiser draws whatever waypoints this file produces (it reads them from
 * the firmware's startup header), so changing the track here is all that's needed.
 */

/* Spacing between adjacent waypoints along the centreline, metres.
 * Smaller = smoother curves but more waypoints (watch MAX_WAYPOINTS). */
#define WAYPOINT_SPACING_M  3.0f


/* --- Turtle state used while building the track --- */
typedef struct {
    Track *track;
    float  x, y;        /* current pen position, metres */
    float  heading;     /* current direction, radians (0 = +x, pi/2 = +y) */
} Turtle;


static void turtle_emit(Turtle *t)
{
    if (t->track->count < MAX_WAYPOINTS) {
        t->track->points[t->track->count].x = t->x;
        t->track->points[t->track->count].y = t->y;
        t->track->count++;
    }
}

/* Lay down a straight segment of the given length in the current heading. */
static void straight(Turtle *t, float length)
{
    int   steps = (int)(length / WAYPOINT_SPACING_M);
    float step  = (steps > 0) ? length / steps : length;
    int   i;
    for (i = 0; i < steps; i++) {
        t->x += step * cosf(t->heading);
        t->y += step * sinf(t->heading);
        turtle_emit(t);
    }
}

/*
 * Lay down a constant-radius arc.
 *   radius_m    : turn radius (centre is to the left of travel for a left turn)
 *   angle_deg   : signed sweep.  +ve = left (CCW), -ve = right (CW).
 */
static void arc(Turtle *t, float radius_m, float angle_deg)
{
    float angle_rad = angle_deg * PI / 180.0f;
    float arc_len   = fabsf(angle_rad) * radius_m;
    int   steps     = (int)(arc_len / WAYPOINT_SPACING_M);
    if (steps < 1) steps = 1;

    float dtheta = angle_rad / steps;   /* heading change per step */
    int   i;
    for (i = 0; i < steps; i++) {
        /* Advance along a short chord, then rotate the heading. */
        t->heading += dtheta * 0.5f;
        t->x += (radius_m * fabsf(dtheta)) * cosf(t->heading);
        t->y += (radius_m * fabsf(dtheta)) * sinf(t->heading);
        t->heading += dtheta * 0.5f;
        turtle_emit(t);
    }
}


void track_init(Track *track)
{
    Turtle t = { track, 0.0f, 0.0f, PI / 2.0f };  /* start at origin heading north */

    track->count         = 0;
    track->current_index = 0;
    track->lap_count     = 0;

    /*
     * Technical infield layout.  Walk it as a loop of straights, hairpins and
     * esses.  The signed arc angles sum to +360 deg (one CCW lap) and the corner
     * angles were tuned so the turtle returns to the start point still heading
     * north, closing the lap exactly.  ~90 waypoints at 3 m spacing.
     *
     * To author a different track: edit the segments below.  Keep the signed arc
     * angles summing to ~+/-360 for a closed loop, and aim to finish near (0,0)
     * heading north.  The renderer copes with any crossings or tight turns.
     */
    turtle_emit(&t);              /* emit the start point at (0,0) heading north */

    straight(&t,  40.0f);         /* start/finish straight                   */
    arc(&t, 10.0f,   61.1f);      /* turn 1: left                            */
    straight(&t,  12.0f);
    arc(&t,  7.0f,  153.1f);      /* turn 2: hairpin left                    */
    straight(&t,  16.0f);

    /* Esses: left-right-left-right flick */
    arc(&t,  9.0f,  -75.0f);
    arc(&t,  9.0f,   75.0f);
    arc(&t,  9.0f,  -75.0f);
    arc(&t,  9.0f,   75.0f);

    straight(&t,  14.0f);
    arc(&t,  7.0f,  164.7f);      /* turn 3: hairpin left                    */
    straight(&t,  24.0f);
    arc(&t, 12.0f,  110.2f);      /* turn 4: long left                       */
    straight(&t,  12.0f);
    arc(&t,  9.0f,  149.6f);      /* turn 5: tight left                      */
    straight(&t,  18.0f);
    arc(&t, 11.0f,   81.4f);      /* turn 6: sweep left onto start straight  */
}


void track_update(Track *track, float car_x, float car_y)
{
    TrackPoint target = track->points[track->current_index];
    float dx = car_x - target.x;
    float dy = car_y - target.y;
    float dist_sq = dx * dx + dy * dy;
    float capture_sq = WAYPOINT_CAPTURE_RADIUS_M * WAYPOINT_CAPTURE_RADIUS_M;

    if (dist_sq < capture_sq) {
        track->current_index++;
        if (track->current_index >= track->count) {
            track->current_index = 0;
            track->lap_count++;
        }
    }
}


TrackPoint track_get_target(const Track *track)
{
    return track->points[track->current_index];
}


void track_get_bounds(const Track *track,
                      float *min_x, float *max_x,
                      float *min_y, float *max_y)
{
    int i;
    *min_x =  FLT_MAX;
    *max_x = -FLT_MAX;
    *min_y =  FLT_MAX;
    *max_y = -FLT_MAX;

    for (i = 0; i < track->count; i++) {
        if (track->points[i].x < *min_x) *min_x = track->points[i].x;
        if (track->points[i].x > *max_x) *max_x = track->points[i].x;
        if (track->points[i].y < *min_y) *min_y = track->points[i].y;
        if (track->points[i].y > *max_y) *max_y = track->points[i].y;
    }

    /* Add a margin so the track doesn't touch the edge of the terminal */
    *min_x -= 8.0f;
    *max_x += 8.0f;
    *min_y -= 8.0f;
    *max_y += 8.0f;
}
