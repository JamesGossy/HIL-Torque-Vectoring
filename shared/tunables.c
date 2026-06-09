/*
 * shared/tunables.c
 *
 * Single owner of every runtime-tunable gain: the values the parameter sweep
 * optimises. Each is a global initialised to its default literal, optionally
 * overridden from a TUNE_<NAME> environment variable at startup. See tunables.h
 * for the rationale (the sweep builds ONCE and varies these at runtime instead
 * of recompiling per candidate).
 *
 * These are the gains that used to be #ifndef macros in parameters_config.h /
 * lqr_steer.c / path_planning.c. They live here, in one place, so there is no
 * macro/global duplication and any module just extern-references the globals it
 * uses. Fixed, non-tuned constants (motor limits, control period, deadbands,
 * geometry-derived gains) remain compile-time #defines in constants_config.h
 * (the renamed parameters_config.h).
 *
 * Defaults below are the committed tuned set (fsg2024 ~24.8s, fse2024 ~18.9s,
 * both clean and robust to +-3%). Re-tune with tools/tool_smart_sweep_lqr_multi.py
 * and paste the winning values here.
 */

#include "tunables.h"
#include <stdlib.h>

/* ---- defaults (committed tuned set) ---- */

/* DRIVER: speed planner */
float g_MAX_LATERAL_ACCEL_MS2  = 12.6154f;
float g_LAT_GRIP_REF_MS2       = 16.5044f;
float g_GG_ACCEL_MS2           = 8.7898f;
float g_PLANNER_DOWNFORCE_FRAC = 0.5670f;

/* DRIVER: racing-line shape (path_planning.c) */
float g_RACING_MARGIN   = 0.2587f;
float g_PP_GRIP_ACCEL   = 10.0000f;
float g_PP_MIN_RADIUS_M = 6.0329f;

/* DRIVER: LQR steering cost weights (lqr_steer.c) */
float g_LQR_Q_E1  = 10.0000f;
float g_LQR_Q_E1D = 1.0432f;
float g_LQR_Q_E2  = 9.7510f;
float g_LQR_Q_E2D = 0.4511f;
float g_LQR_R     = 2.8632f;
float g_LQR_KI    = 7.9193f;
float g_LQR_I_MAX = 0.3000f;

/* ECU: torque vectoring */
float g_KP_YAW_DEFAULT = 86.2440f;
float g_TV_KFF         = 10.3635f;

/* Read a float from environment variable `name`, or return `fallback` if it is
 * unset or unparseable. */
static float getenvf(const char *name, float fallback)
{
    const char *s = getenv(name);
    if (!s || !*s) return fallback;
    char *end = 0;
    float v   = strtof(s, &end);
    return (end == s) ? fallback : v;
}

void tunables_init_from_env(void)
{
    g_MAX_LATERAL_ACCEL_MS2  = getenvf("TUNE_MAX_LATERAL_ACCEL_MS2", g_MAX_LATERAL_ACCEL_MS2);
    g_LAT_GRIP_REF_MS2       = getenvf("TUNE_LAT_GRIP_REF_MS2", g_LAT_GRIP_REF_MS2);
    g_GG_ACCEL_MS2           = getenvf("TUNE_GG_ACCEL_MS2", g_GG_ACCEL_MS2);
    g_PLANNER_DOWNFORCE_FRAC = getenvf("TUNE_PLANNER_DOWNFORCE_FRAC", g_PLANNER_DOWNFORCE_FRAC);

    g_RACING_MARGIN   = getenvf("TUNE_RACING_MARGIN", g_RACING_MARGIN);
    g_PP_GRIP_ACCEL   = getenvf("TUNE_PP_GRIP_ACCEL", g_PP_GRIP_ACCEL);
    g_PP_MIN_RADIUS_M = getenvf("TUNE_PP_MIN_RADIUS_M", g_PP_MIN_RADIUS_M);

    g_LQR_Q_E1  = getenvf("TUNE_LQR_Q_E1", g_LQR_Q_E1);
    g_LQR_Q_E1D = getenvf("TUNE_LQR_Q_E1D", g_LQR_Q_E1D);
    g_LQR_Q_E2  = getenvf("TUNE_LQR_Q_E2", g_LQR_Q_E2);
    g_LQR_Q_E2D = getenvf("TUNE_LQR_Q_E2D", g_LQR_Q_E2D);
    g_LQR_R     = getenvf("TUNE_LQR_R", g_LQR_R);
    g_LQR_KI    = getenvf("TUNE_LQR_KI", g_LQR_KI);
    g_LQR_I_MAX = getenvf("TUNE_LQR_I_MAX", g_LQR_I_MAX);

    g_KP_YAW_DEFAULT = getenvf("TUNE_KP_YAW_DEFAULT", g_KP_YAW_DEFAULT);
    g_TV_KFF         = getenvf("TUNE_TV_KFF", g_TV_KFF);
}
