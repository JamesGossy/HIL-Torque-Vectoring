#ifndef TUNABLES_H
#define TUNABLES_H

/*
 * shared/tunables.h
 *
 * Runtime-overridable tuning gains. Every gain the sweep optimises has a global
 * here, initialised to its compile-time default (the #ifndef macro in
 * parameters_config.h / lqr_steer.c / path_planning.c) and optionally overridden
 * from an environment variable at startup.
 *
 * Why: the parameter sweep used to inject each candidate's gains as -D macros,
 * forcing a full gcc rebuild PER CANDIDATE - ~0.96s of compiling to hide a
 * ~0.08s lap. 86% of sweep wall-time was the compiler. With these globals the
 * sweep builds the binary ONCE and each candidate just sets env vars and runs,
 * turning a ~14-minute sweep into ~15 seconds.
 *
 * The compile-time defaults are unchanged: a normal build (and any -D override,
 * as CI uses) still bakes the values in; tunables_init_from_env() only replaces
 * a global if its TUNE_<NAME> env var is present. The real ECU/sim are
 * unaffected unless an env var is deliberately set.
 *
 * Each global is read where the code used to use the macro directly. All such
 * sites run AFTER main() (the LQR gain recomputes lazily on the first steer call;
 * the racing line is built during path-planning init), so a single
 * tunables_init_from_env() at the top of main() precedes every use.
 */

/* DRIVER: speed planner */
extern float g_MAX_LATERAL_ACCEL_MS2;
extern float g_LAT_GRIP_REF_MS2;

/* DRIVER: friction-circle / GG budget */
extern float g_GG_ACCEL_MS2;
extern float g_PLANNER_DOWNFORCE_FRAC;

/* DRIVER: racing line (path_planning.c) */
extern float g_RACING_MARGIN;
extern float g_PP_GRIP_ACCEL;
extern float g_PP_MIN_RADIUS_M;

/* DRIVER: LQR steering cost weights (lqr_steer.c) */
extern float g_LQR_Q_E1;
extern float g_LQR_Q_E1D;
extern float g_LQR_Q_E2;
extern float g_LQR_Q_E2D;
extern float g_LQR_R;
extern float g_LQR_KI;
extern float g_LQR_I_MAX;

/* ECU: torque vectoring */
extern float g_KP_YAW_DEFAULT;
extern float g_TV_KFF;

/*
 * Override any global whose TUNE_<NAME> environment variable is set (e.g.
 * TUNE_LQR_R=5.94). Call once at the top of main() in every entry point. Safe to
 * call when no env vars are set - it is then a no-op and the compile-time
 * defaults stand.
 */
void tunables_init_from_env(void);

#endif /* TUNABLES_H */
