#include "../include/visualiser.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/*
 * visualiser.c
 *
 * Draws the track and car in the terminal using plain printf and ANSI codes.
 *
 * How the coordinate mapping works:
 *
 *   The track exists in world space (metres, floating point).
 *   The terminal has VIS_COLS x VIS_ROWS character cells.
 *
 *   We compute a scale factor once at startup so the whole track fits in the
 *   grid with some margin. Then for every world coordinate (x, y), the
 *   corresponding grid cell is:
 *
 *     col = (x - world_min_x) / (world_max_x - world_min_x) * VIS_COLS
 *     row = (1 - (y - world_min_y) / (world_max_y - world_min_y)) * VIS_ROWS
 *
 *   The row is flipped (1 - ...) because terminal rows increase downward,
 *   but Y in world space increases upward.
 *
 * The track grid is pre-rendered once at startup. Each frame, we copy the
 * static grid, stamp the car's position onto it, and print the whole thing.
 * Then we print the status line below.
 *
 * ANSI escape codes used:
 *   \033[H   -- move cursor to top-left (home position)
 *   \033[2J  -- clear the screen (only used once at startup)
 */


static int world_to_col(const Visualiser *vis, float x)
{
    float t = (x - vis->world_min_x) / (vis->world_max_x - vis->world_min_x);
    int col = (int)(t * (VIS_COLS - 1));
    if (col < 0)          col = 0;
    if (col >= VIS_COLS)  col = VIS_COLS - 1;
    return col;
}

static int world_to_row(const Visualiser *vis, float y)
{
    float t = (y - vis->world_min_y) / (vis->world_max_y - vis->world_min_y);
    int row = (int)((1.0f - t) * (VIS_ROWS - 1));
    if (row < 0)          row = 0;
    if (row >= VIS_ROWS)  row = VIS_ROWS - 1;
    return row;
}


void visualiser_init(Visualiser *vis, const Track *track)
{
    int row, col, i;

    track_get_bounds(track,
                     &vis->world_min_x, &vis->world_max_x,
                     &vis->world_min_y, &vis->world_max_y);

    /* Fill the grid with spaces */
    for (row = 0; row < VIS_ROWS; row++) {
        for (col = 0; col < VIS_COLS; col++) {
            vis->grid[row][col] = ' ';
        }
        vis->grid[row][VIS_COLS] = '\0';
    }

    /* --- Draw the track ---
     * For each waypoint, mark a small patch of cells around its grid position
     * as track surface ('.'), and the outermost ring as boundary ('#').
     *
     * We do this by iterating each waypoint and each nearby cell and checking
     * if the world distance is within the track width. */
    float x_scale = (vis->world_max_x - vis->world_min_x) / (float)(VIS_COLS - 1);
    float y_scale = (vis->world_max_y - vis->world_min_y) / (float)(VIS_ROWS - 1);

    /* The terminal cells are not square -- a character is roughly twice as tall
     * as it is wide. We account for this when computing distances. */
    float aspect = x_scale / (y_scale > 0.0f ? y_scale : 1.0f);

    for (i = 0; i < track->count; i++) {
        int centre_col = world_to_col(vis, track->points[i].x);
        int centre_row = world_to_row(vis, track->points[i].y);

        /* Search a patch of cells around the waypoint */
        int search_r = 6;
        for (row = centre_row - search_r; row <= centre_row + search_r; row++) {
            for (col = centre_col - search_r; col <= centre_col + search_r; col++) {
                if (row < 0 || row >= VIS_ROWS) continue;
                if (col < 0 || col >= VIS_COLS) continue;

                /* Convert this cell back to world coordinates */
                float wx = vis->world_min_x + col * x_scale;
                float wy = vis->world_max_y - row * y_scale;  /* note: row is flipped */

                /* World-space distance from this cell to the waypoint */
                float dx = wx - track->points[i].x;
                float dy = (wy - track->points[i].y) * aspect;
                float dist = sqrtf(dx * dx + dy * dy);

                if (dist < TRACK_DRAW_WIDTH_M - 1.5f) {
                    if (vis->grid[row][col] == ' ') {
                        vis->grid[row][col] = '.';
                    }
                } else if (dist < TRACK_DRAW_WIDTH_M) {
                    vis->grid[row][col] = '#';
                }
            }
        }
    }

    /* Clear the screen once */
    printf("\033[2J");
}


void visualiser_draw(const Visualiser    *vis,
                     const VehicleState  *state,
                     const WheelTorques  *torques,
                     int                  tv_enabled,
                     float                kp_yaw,
                     int                  lap_count,
                     float                elapsed_s)
{
    /* Copy the static track grid into a working buffer */
    char frame[VIS_ROWS][VIS_COLS + 1];
    int row, col;
    for (row = 0; row < VIS_ROWS; row++) {
        memcpy(frame[row], vis->grid[row], VIS_COLS + 1);
    }

    /* Stamp the car's position onto the frame */
    int car_col = world_to_col(vis, state->x);
    int car_row = world_to_row(vis, state->y);
    frame[car_row][car_col] = 'O';

    /* Move cursor to home without clearing (avoids flicker) */
    printf("\033[H");

    /* Print the frame */
    for (row = 0; row < VIS_ROWS; row++) {
        /* Print border */
        putchar('|');
        for (col = 0; col < VIS_COLS; col++) {
            putchar(frame[row][col]);
        }
        puts("|");
    }

    /* Print a separator */
    for (col = 0; col < VIS_COLS + 2; col++) putchar('-');
    putchar('\n');

    /* --- Status line --- */
    float speed_kmh = state->velocity * 3.6f;
    float yaw_degs  = state->yaw_rate * 57.296f;  /* rad/s to deg/s */
    int   mins      = (int)(elapsed_s / 60.0f);
    int   secs      = (int)(elapsed_s) % 60;

    printf("  Speed: %5.1f km/h    Yaw rate: %+6.1f deg/s    Lap: %d    Time: %d:%02d\n",
           speed_kmh, yaw_degs, lap_count, mins, secs);

    printf("  Torques (Nm)  FL: %5.1f   FR: %5.1f   RL: %5.1f   RR: %5.1f\n",
           torques->fl, torques->fr, torques->rl, torques->rr);

    printf("  TV: %s    Kp: %.1f    [t] toggle   [ ] adjust Kp   [q] quit\n",
           tv_enabled ? "ON " : "OFF", kp_yaw);

    fflush(stdout);
}
