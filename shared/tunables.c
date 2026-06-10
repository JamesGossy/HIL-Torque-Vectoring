/* shared/tunables.c - every controller gain as a g_* global. SWEPT gains take a
 * TUNE_<NAME> env override and are in the sweep's PARAMS; the two FIXED gains are
 * not performance choices. Physical/derived constants live in vehicle_config.h. */

#include "tunables.h"
#include "vehicle_config.h" /* caps for clamping the tunable search depths */
#include <stdlib.h>

/* ---- SWEPT: grip / steering / racing line ---- */
float g_GRIP_USE            = 0.90f;   /* fraction of the physical peak lateral grip   */
float g_K_STANLEY           = 8.0f;    /* Stanley steering cross-track gain            */
float g_K_DAMP              = 0.30f;   /* Stanley yaw-rate damping gain                */
float g_RACING_MARGIN       = 0.4000f; /* racing-line corridor safety margin           */
float g_PP_RADIUS_FACTOR    = 1.6f;    /* opens the kinematic radius floor for slip    */
float g_MAX_STEER_RAD       = 1.7f;    /* steering reference limit (driver cap)        */
float g_MAX_STEER_RATE_RADS = 8.0f;    /* steering slew limit, rad/s                   */
float g_STEER_SAT_FRAC      = 0.7f;    /* lock fraction above which throttle fades     */

/* ---- SWEPT: speed planner / longitudinal control ---- */
float g_SPEED_PLAN_HORIZON_M = 80.0f;  /* corner-scan lookahead, m                    */
int g_SPEED_PLAN_STEPS       = 40;     /* scan depth (clamped to the cap)             */
float g_MAX_BRAKE_DECEL_MS2  = 5.6f;   /* braking-effort cap, m/s^2 (below regen lim) */
float g_SPEED_KP_NM          = 800.0f; /* throttle P-gain, Nm per m/s             */
float g_BRAKE_KP_NM          = 16.2f;  /* brake P-gain, Nm per m/s                */
float g_SPEED_KI_NM          = 400.0f; /* throttle I-gain, Nm per m/s per s       */
float g_SPEED_I_MAX_NM       = 250.0f; /* throttle integral clamp, Nm             */
int g_NEAREST_SEARCH_BACK    = 3;      /* segments searched back (clamped to cap)  */
int g_NEAREST_SEARCH_FWD     = 30;     /* segments searched fwd (clamped to cap)   */

/* ---- SWEPT: cone boundary safety net ---- */
float g_BOUNDARY_WARN_M      = 1.0f;  /* steer away from a cone within this range  */
float g_BOUNDARY_CORR_GAIN   = 0.30f; /* max boundary steer correction, rad        */
float g_BOUNDARY_SLOW_M      = 1.0f;  /* slow down within this range of a cone     */
float g_BOUNDARY_SLOW_FACTOR = 0.6f;  /* speed floor multiplier at the cone face   */

/* ---- SWEPT: torque vectoring ---- */
float g_KP_YAW             = 86.2440f; /* master TV yaw gain, Nm per rad/s            */
float g_TV_KI_FRAC         = 2.5f;     /* integral gain as a fraction of KP_YAW       */
float g_TV_KD_FRAC         = 0.05f;    /* derivative gain as a fraction of KP_YAW     */
float g_TV_KFF_FRAC        = 0.12f;    /* feedforward gain as a fraction of KP_YAW    */
float g_TV_I_MAX_FRAC      = 0.408f;   /* integral bias cap as a fraction of motor pk */
float g_TV_SPEED_REF_MS    = 12.0f;    /* neutral speed for the gain scaling, m/s     */
float g_TV_WHEEL_YAW_TRUST = 0.25f;    /* weight on the wheel-speed yaw estimate (0-1) */

/* ---- FIXED: not performance choices, not swept ---- */
float g_TV_YAW_DEADBAND = 0.03f; /* sensor-noise floor, rad/s                     */
float g_TV_K_US         = 0.06f; /* empirical understeer term (linear deriv ~0)   */

/* Read a float from env var `name`, or `fallback` if unset/unparseable. */
static float getenvf(const char *name, float fallback)
{
    const char *s = getenv(name);
    if (!s || !*s) return fallback;
    char *end = 0;
    float v   = strtof(s, &end);
    return (end == s) ? fallback : v;
}

/* Read an int (float-parsed so "40" works), clamped to [lo, hi]. */
static int getenvi_clamped(const char *name, int fallback, int lo, int hi)
{
    int v = (int)(getenvf(name, (float)fallback) + 0.5f);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return v;
}

void tunables_init_from_env(void)
{
    g_GRIP_USE         = getenvf("TUNE_GRIP_USE", g_GRIP_USE);
    g_K_STANLEY        = getenvf("TUNE_K_STANLEY", g_K_STANLEY);
    g_K_DAMP           = getenvf("TUNE_K_DAMP", g_K_DAMP);
    g_RACING_MARGIN    = getenvf("TUNE_RACING_MARGIN", g_RACING_MARGIN);
    g_PP_RADIUS_FACTOR = getenvf("TUNE_PP_RADIUS_FACTOR", g_PP_RADIUS_FACTOR);

    g_MAX_STEER_RAD       = getenvf("TUNE_MAX_STEER_RAD", g_MAX_STEER_RAD);
    g_MAX_STEER_RATE_RADS = getenvf("TUNE_MAX_STEER_RATE_RADS", g_MAX_STEER_RATE_RADS);
    g_STEER_SAT_FRAC      = getenvf("TUNE_STEER_SAT_FRAC", g_STEER_SAT_FRAC);

    g_SPEED_PLAN_HORIZON_M = getenvf("TUNE_SPEED_PLAN_HORIZON_M", g_SPEED_PLAN_HORIZON_M);
    g_SPEED_PLAN_STEPS
        = getenvi_clamped("TUNE_SPEED_PLAN_STEPS", g_SPEED_PLAN_STEPS, 4, SPEED_PLAN_STEPS_CAP);
    g_MAX_BRAKE_DECEL_MS2 = getenvf("TUNE_MAX_BRAKE_DECEL_MS2", g_MAX_BRAKE_DECEL_MS2);

    g_SPEED_KP_NM    = getenvf("TUNE_SPEED_KP_NM", g_SPEED_KP_NM);
    g_BRAKE_KP_NM    = getenvf("TUNE_BRAKE_KP_NM", g_BRAKE_KP_NM);
    g_SPEED_KI_NM    = getenvf("TUNE_SPEED_KI_NM", g_SPEED_KI_NM);
    g_SPEED_I_MAX_NM = getenvf("TUNE_SPEED_I_MAX_NM", g_SPEED_I_MAX_NM);

    g_NEAREST_SEARCH_BACK = getenvi_clamped(
        "TUNE_NEAREST_SEARCH_BACK", g_NEAREST_SEARCH_BACK, 1, NEAREST_SEARCH_BACK_CAP);
    g_NEAREST_SEARCH_FWD = getenvi_clamped(
        "TUNE_NEAREST_SEARCH_FWD", g_NEAREST_SEARCH_FWD, 2, NEAREST_SEARCH_FWD_CAP);

    g_BOUNDARY_WARN_M      = getenvf("TUNE_BOUNDARY_WARN_M", g_BOUNDARY_WARN_M);
    g_BOUNDARY_CORR_GAIN   = getenvf("TUNE_BOUNDARY_CORR_GAIN", g_BOUNDARY_CORR_GAIN);
    g_BOUNDARY_SLOW_M      = getenvf("TUNE_BOUNDARY_SLOW_M", g_BOUNDARY_SLOW_M);
    g_BOUNDARY_SLOW_FACTOR = getenvf("TUNE_BOUNDARY_SLOW_FACTOR", g_BOUNDARY_SLOW_FACTOR);

    g_KP_YAW             = getenvf("TUNE_KP_YAW", g_KP_YAW);
    g_TV_KI_FRAC         = getenvf("TUNE_TV_KI_FRAC", g_TV_KI_FRAC);
    g_TV_KD_FRAC         = getenvf("TUNE_TV_KD_FRAC", g_TV_KD_FRAC);
    g_TV_KFF_FRAC        = getenvf("TUNE_TV_KFF_FRAC", g_TV_KFF_FRAC);
    g_TV_I_MAX_FRAC      = getenvf("TUNE_TV_I_MAX_FRAC", g_TV_I_MAX_FRAC);
    g_TV_SPEED_REF_MS    = getenvf("TUNE_TV_SPEED_REF_MS", g_TV_SPEED_REF_MS);
    g_TV_WHEEL_YAW_TRUST = getenvf("TUNE_TV_WHEEL_YAW_TRUST", g_TV_WHEEL_YAW_TRUST);
}
