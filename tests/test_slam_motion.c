/* Focused probe: does cone fusion HELP a moving car vs pure dead-reckoning?
 * Drives a known straight-line trajectory at speed, feeds true odometry plus
 * noisy cone observations, and compares fused pose error to dead-reckoned. */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../ECU_Firmware/include/slam.h"
#include "../shared/tunables.h"

static float wrap_pi(float a)
{
    while (a > 3.14159265f)
        a -= 6.2831853f;
    while (a < -3.14159265f)
        a += 6.2831853f;
    return a;
}

/* Cones lining a constant-radius bend: pairs at +/-2 m across a 25 m-radius arc. */
#define NCONES 60
static float cones[NCONES][3];
#define ARC_R 25.0f

static void build_cones(void)
{
    int k = 0;
    for (int i = 0; i < NCONES / 2; i++) {
        float a          = i * 0.08f; /* radians around the arc */
        float cx         = ARC_R * sinf(a);
        float cy         = ARC_R * (1.0f - cosf(a)); /* centre of corridor */
        float nx = sinf(a), ny = cosf(a);            /* outward normal-ish */
        cones[k][0]      = cx - 2.0f * (-ny); /* left */
        cones[k][1]      = cy - 2.0f * (nx);
        cones[k++][2]    = (float)CONE_COLOR_LEFT;
        cones[k][0]      = cx + 2.0f * (-ny); /* right */
        cones[k][1]      = cy + 2.0f * (nx);
        cones[k++][2]    = (float)CONE_COLOR_RIGHT;
    }
}

/* Perfect-geometry scan with optional Gaussian noise (deterministic). */
static unsigned rng = 12345u;
static float nrand(void)
{
    rng     = rng * 1664525u + 1013904223u;
    float u1 = (float)((rng >> 8) & 0xffffff) / (float)0x1000000;
    rng     = rng * 1664525u + 1013904223u;
    float u2 = (float)((rng >> 8) & 0xffffff) / (float)0x1000000;
    if (u1 < 1e-7f) u1 = 1e-7f;
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static void scan_from(ConeScan *s, float x, float y, float th, int noisy)
{
    s->count = 0;
    for (int i = 0; i < NCONES && s->count < MAX_OBS_PER_TICK; i++) {
        float dx = cones[i][0] - x, dy = cones[i][1] - y;
        float r  = sqrtf(dx * dx + dy * dy);
        if (r > g_SENSOR_RANGE_M) continue;
        float b = wrap_pi(atan2f(dy, dx) - th);
        if (fabsf(b) > g_SENSOR_FOV_RAD) continue;
        if (noisy) {
            r += g_SENSOR_RANGE_SIGMA_M * nrand();
            b += g_SENSOR_BEARING_SIGMA_RAD * nrand();
        }
        s->obs[s->count].range   = r;
        s->obs[s->count].bearing = b;
        s->obs[s->count].color   = (int)cones[i][2];
        s->count++;
    }
}

int main(void)
{
    tunables_init_from_env();
    build_cones();

    const float dt = 0.01f, v = 15.0f; /* fast straight */
    const int N    = 300;              /* 3 s, 45 m */

    /* true pose */
    float tx = 0, ty = 0, th = 0;

    static SlamState slam; /* ~1.45 MB each; keep off the stack */
    slam_init(&slam, 0, 0, 0);

    /* dead-reckoning reference (predict only, no updates) */
    static SlamState dr;
    slam_init(&dr, 0, 0, 0);

    SensorData sd = { 0 };
    sd.velocity   = v;
    sd.yaw_rate   = v / ARC_R; /* follow the bend */

    double fused_sq = 0, dr_sq = 0;
    for (int i = 0; i < N; i++) {
        tx += v * cosf(th) * dt;
        ty += v * sinf(th) * dt;
        th = wrap_pi(th + sd.yaw_rate * dt);

        slam_predict(&slam, &sd, dt);
        ConeScan sc;
        scan_from(&sc, tx, ty, th, 1);
        slam_update(&slam, &sc);

        slam_predict(&dr, &sd, dt); /* no update */

        float ex = slam.mu[0] - tx, ey = slam.mu[1] - ty;
        float dx = dr.mu[0] - tx, dy = dr.mu[1] - ty;
        fused_sq += ex * ex + ey * ey;
        dr_sq += dx * dx + dy * dy;

        if (i % 50 == 0)
            fprintf(stderr, "i=%3d fused_err=%.3f dr_err=%.3f nL=%d\n", i,
                sqrtf(ex * ex + ey * ey), sqrtf(dx * dx + dy * dy), slam.n_land);
    }
    float fused_rmse = sqrtf((float)(fused_sq / N));
    float dr_rmse    = sqrtf((float)(dr_sq / N));
    fprintf(stderr, "fused RMSE=%.3f  dead-reckon RMSE=%.3f  landmarks=%d\n", fused_rmse, dr_rmse,
        slam.n_land);
    /* Fusion must not be worse than dead reckoning. With true odometry dr is ~0,
     * so fusion should also be tiny. */
    fprintf(stderr, "%s\n", (fused_rmse <= dr_rmse + 0.2f) ? "PASS fusion-not-worse"
                                                           : "FAIL fusion-degrades");
    return (fused_rmse <= dr_rmse + 0.2f) ? 0 : 1;
}
