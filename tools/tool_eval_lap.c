/*
 * Headless lap evaluator. Runs the per-tick loop as fast as possible and
 * measures cross-track error, cone contacts and lap time. Prints a
 * machine-readable RESULT line for sweeps and CI. Build with `make eval`.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "../HIL_Firmware/include/vehicle_model.h"
#include "../HIL_Firmware/include/track_parser.h"
#include "../HIL_Firmware/include/motion_control.h"
#include "../shared/tv_interface.h"
#include "../shared/tunables.h"
#include "../ECU_Firmware/include/torque_vectoring.h"

#define DT 0.01f

/* Nearest distance from (x,y) to the racing-line polyline. */
static float dist_to_line(const Track *t, float x, float y, int *near_idx)
{
    float best = 1e18f;
    int bi     = 0;
    for (int i = 0; i < t->count; i++) {
        int j    = (i + 1) % t->count;
        float ax = t->points[i].x, ay = t->points[i].y;
        float ex = t->points[j].x - ax, ey = t->points[j].y - ay;
        float l2 = ex * ex + ey * ey;
        float s  = l2 > 1e-9f ? ((x - ax) * ex + (y - ay) * ey) / l2 : 0.0f;
        if (s < 0) s = 0;
        if (s > 1) s = 1;
        float cx = ax + s * ex, cy = ay + s * ey;
        float dx = x - cx, dy = y - cy, d2 = dx * dx + dy * dy;
        if (d2 < best) {
            best = d2;
            bi   = i;
        }
    }
    if (near_idx) *near_idx = bi;
    return sqrtf(best);
}

#define CONE_CLEARANCE_M 0.25f /* below the ideal line's ~0.34m apex pass, so only real wides flag */
/* Returns 1 if the car CG is within CONE_CLEARANCE_M of any cone. */
static int off_track(const Track *t, float x, float y)
{
    for (int i = 0; i < t->left_count; i++) {
        float dx = t->left_cones[i].x - x, dy = t->left_cones[i].y - y;
        if (dx * dx + dy * dy < CONE_CLEARANCE_M * CONE_CLEARANCE_M) return 1;
    }
    for (int i = 0; i < t->right_count; i++) {
        float dx = t->right_cones[i].x - x, dy = t->right_cones[i].y - y;
        if (dx * dx + dy * dy < CONE_CLEARANCE_M * CONE_CLEARANCE_M) return 1;
    }
    return 0;
}

/* Runs one capped lap evaluation and prints the metrics and RESULT line. */
int main(void)
{
    Track track;
    VehicleState state;
    WheelTorques torques = { 0 };
    SensorData sensors   = { 0 };

    tunables_init_from_env(); /* apply TUNE_* overrides before the racing line is built below */

    track_init(&track);

    float ih = atan2f(track.points[1].y - track.points[0].y, track.points[1].x - track.points[0].x);
    vehicle_model_init(&state, track.points[0].x, track.points[0].y, ih);

    motion_control_reset();
    torque_vectoring_reset();

    /* Precompute per-waypoint curvature via three-point Menger on +/-2 waypoints. */
    int n        = track.count;
    float *kappa = malloc(sizeof(float) * n);
    for (int i = 0; i < n; i++) {
        int p = (i - 2 + n) % n, c = i, q = (i + 2) % n;
        float ax = track.points[p].x, ay = track.points[p].y;
        float bx = track.points[c].x, by = track.points[c].y;
        float cx = track.points[q].x, cy = track.points[q].y;
        float abx = bx - ax, aby = by - ay, bcx = cx - bx, bcy = cy - by, cax = ax - cx,
              cay = ay - cy;
        float ab = sqrtf(abx * abx + aby * aby), bc = sqrtf(bcx * bcx + bcy * bcy),
              ca  = sqrtf(cax * cax + cay * cay);
        float crs = fabsf(abx * bcy - aby * bcx);
        kappa[i]  = (ab < 0.01f || bc < 0.01f || ca < 0.01f || crs < 1e-6f)
            ? 0.0f
            : 2.0f * crs / (ab * bc * ca);
    }

    int trace             = (getenv("TRACE") != NULL);
    int max_ticks         = 5000; /* 50 s cap */
    float worst_cte       = 0.0f;
    float worst_cte_sharp = 0.0f; /* worst CTE near a sharp corner (kappa>0.15) */
    int worst_idx         = -1;
    double sum_cte        = 0.0;
    long cnt              = 0;
    int sharp_violations  = 0; /* ticks with CTE>2.0m near a sharp corner */
    int lap_done_tick     = -1;
    int start_lap         = track.lap_count;
    int moved_away        = 0;
    int offtrack_ticks    = 0;

    for (int tick = 1; tick <= max_ticks; tick++) {
        float target_speed = 0.0f;
        float dq           = motion_control_update(&state, &track, &target_speed);

        sensors.yaw_rate              = state.yaw_rate;
        sensors.velocity              = state.velocity;
        sensors.steering_angle        = state.steering;
        sensors.driver_torque         = dq;
        const float R2W               = (2.0f * 3.14159265358979f) / (GEAR_RATIO * 60.0f);
        sensors.wheel_speed[WHEEL_FL] = state.wheelspeed[WHEEL_FL] * R2W;
        sensors.wheel_speed[WHEEL_FR] = state.wheelspeed[WHEEL_FR] * R2W;
        sensors.wheel_speed[WHEEL_RL] = state.wheelspeed[WHEEL_RL] * R2W;
        sensors.wheel_speed[WHEEL_RR] = state.wheelspeed[WHEEL_RR] * R2W;

        torque_vectoring_update(&sensors, dq, g_KP_YAW, &torques);
        vehicle_model_update(&state, &torques, DT);
        track_update(&track, state.x, state.y);

        int ni;
        float cte = dist_to_line(&track, state.x, state.y, &ni);
        sum_cte += cte;
        cnt++;
        if (cte > worst_cte) {
            worst_cte = cte;
            worst_idx = ni;
        }

        int is_sharp = (kappa[ni] > 0.15f);
        if (is_sharp && cte > worst_cte_sharp) worst_cte_sharp = cte;
        if (is_sharp && cte > 2.0f) sharp_violations++;

        if (off_track(&track, state.x, state.y)) offtrack_ticks++;

        if (trace && (tick % 10 == 0))
            printf("t=%5.2f wp=%3d kappa=%.3f cte=%.2f v=%4.1f vtgt=%4.1f steer=%+.3f beta=%+.3f\n",
                tick * DT, ni, kappa[ni], cte, state.velocity, target_speed, state.steering,
                state.slip_angle);

        float d0 = hypotf(state.x - track.points[0].x, state.y - track.points[0].y);
        if (d0 > 15.0f) moved_away = 1; /* must leave the start before a lap counts */
        if (moved_away && track.lap_count > start_lap && lap_done_tick < 0) lap_done_tick = tick;
    }

    printf("=== LAP EVALUATION ===\n");
    printf("waypoints: %d\n", n);
    printf("mean CTE:        %.3f m\n", (float)(sum_cte / cnt));
    printf("worst CTE:       %.3f m  (near waypoint %d, kappa=%.3f 1/m, R=%.1f m)\n", worst_cte,
        worst_idx, worst_idx >= 0 ? kappa[worst_idx] : 0.0f,
        (worst_idx >= 0 && kappa[worst_idx] > 1e-3f) ? 1.0f / kappa[worst_idx] : 999.0f);
    printf("worst CTE @ sharp corner: %.3f m\n", worst_cte_sharp);
    printf("sharp-corner violation ticks (CTE>2m): %d\n", sharp_violations);
    if (lap_done_tick > 0)
        printf("FIRST LAP in %.2f s\n", lap_done_tick * DT);
    else
        printf("LAP NOT COMPLETED within %.1f s (final pos %.1f,%.1f)\n", max_ticks * DT, state.x,
            state.y);
    printf("laps completed in %.1f s: %d\n", max_ticks * DT, track.lap_count - start_lap);
    printf("off-track ticks: %d\n", offtrack_ticks);
    printf("final speed: %.1f km/h\n", state.velocity * 3.6f);

    /* Machine-readable summary for sweeps and CI. lap_s is -1 if no lap completed. */
    printf("RESULT mean_cte=%.3f worst_cte=%.3f worst_cte_sharp=%.3f "
           "sharp_viol=%d offtrack=%d laps=%d lap_s=%.2f\n",
        (float)(sum_cte / cnt), worst_cte, worst_cte_sharp, sharp_violations, offtrack_ticks,
        track.lap_count - start_lap, lap_done_tick > 0 ? lap_done_tick * DT : -1.0f);

    free(kappa);
    return 0; /* always 0, this is a diagnostic judged by its printed numbers */
}
