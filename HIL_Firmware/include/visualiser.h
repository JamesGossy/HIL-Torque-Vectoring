#ifndef VISUALISER_H
#define VISUALISER_H

#include "vehicle_model.h"
#include "track.h"

/*
 * visualiser.h
 *
 * Draws the track and the car in the terminal using ASCII characters and ANSI
 * escape codes. No external libraries are needed.
 *
 * It works by mapping the world coordinate space (metres) to a grid of
 * terminal characters. Each character cell represents a patch of ground.
 *
 * Characters used:
 *   '#'  -- track boundary (the edges of the track)
 *   '.'  -- track surface  (on the racing line)
 *   'O'  -- the car's current position
 *   ' '  -- off-track area
 */


/* Terminal grid size. Should fit in a standard 80x24 terminal.
 * Leave some rows at the bottom for the status display. */
#define VIS_COLS  78
#define VIS_ROWS  30

/* How close a world point needs to be to a track waypoint to count as
 * "on track", in metres. Controls how wide the track line appears. */
#define TRACK_DRAW_WIDTH_M  5.0f


/* Holds the pre-computed grid and coordinate scaling factors.
 * Call visualiser_init() once at startup to fill this in. */
typedef struct {
    char  grid[VIS_ROWS][VIS_COLS + 1];  /* +1 for null terminator per row */
    float world_min_x;
    float world_max_x;
    float world_min_y;
    float world_max_y;
} Visualiser;


/*
 * Set up the visualiser. Computes the coordinate scale from the track bounds
 * and pre-renders the static track grid (track outline and surface).
 *
 * Call this once before the main loop.
 */
void visualiser_init(Visualiser *vis, const Track *track);

/*
 * Draw one frame. Clears to the top of the terminal, renders the track grid
 * with the car at its current position, and prints the status line.
 *
 * Call this at ~10 Hz from the main loop (not every physics tick).
 *
 * tv_enabled  -- 1 if torque vectoring is on, 0 if off
 * kp_yaw      -- current TV gain value (shown in status line)
 * torques     -- current wheel torques (shown in status line)
 * lap_count   -- how many laps completed (shown in status line)
 * elapsed_s   -- total elapsed time in seconds
 */
void visualiser_draw(const Visualiser    *vis,
                     const VehicleState  *state,
                     const WheelTorques  *torques,
                     int                  tv_enabled,
                     float                kp_yaw,
                     int                  lap_count,
                     float                elapsed_s);

#endif /* VISUALISER_H */
