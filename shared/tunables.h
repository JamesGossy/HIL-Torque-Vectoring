#ifndef TUNABLES_H
#define TUNABLES_H

/* Externs for every controller gain. SWEPT gains take a TUNE_<NAME> env override;
 * the two FIXED gains do not. See tunables.c for values and rationale. */

/* ---- SWEPT: grip / steering / racing line ---- */
extern float g_GRIP_USE;            /* fraction of physical peak lateral grip       */
extern float g_K_STANLEY;           /* Stanley cross-track gain                     */
extern float g_K_DAMP;              /* Stanley yaw-rate damping gain                */
extern float g_RACING_MARGIN;       /* racing-line corridor safety margin           */
extern float g_PP_RADIUS_FACTOR;    /* opens the racing-line kinematic radius floor */
extern float g_MAX_STEER_RAD;       /* steering reference limit (driver cap)        */
extern float g_MAX_STEER_RATE_RADS; /* steering slew limit, rad/s                   */
extern float g_STEER_SAT_FRAC;      /* lock fraction above which throttle fades     */

/* ---- SWEPT: speed planner / longitudinal control ---- */
extern float g_SPEED_PLAN_HORIZON_M; /* corner-scan lookahead, m                    */
extern int g_SPEED_PLAN_STEPS;       /* scan depth (clamped to SPEED_PLAN_STEPS_CAP) */
extern float g_MAX_BRAKE_DECEL_MS2;  /* braking-effort cap, m/s^2                    */
extern float g_SPEED_KP_NM;          /* throttle P-gain, Nm per m/s                  */
extern float g_BRAKE_KP_NM;          /* brake P-gain, Nm per m/s                     */
extern float g_SPEED_KI_NM;          /* throttle I-gain, Nm per m/s per s            */
extern float g_SPEED_I_MAX_NM;       /* throttle integral clamp, Nm                  */
extern int g_NEAREST_SEARCH_BACK;    /* segments searched back (clamped to cap)      */
extern int g_NEAREST_SEARCH_FWD;     /* segments searched fwd (clamped to cap)       */

/* ---- SWEPT: cone boundary safety net ---- */
extern float g_BOUNDARY_WARN_M;      /* steer away from a cone within this range, m  */
extern float g_BOUNDARY_CORR_GAIN;   /* max boundary steer correction, rad           */
extern float g_BOUNDARY_SLOW_M;      /* slow down within this range of a cone, m     */
extern float g_BOUNDARY_SLOW_FACTOR; /* speed floor multiplier at the cone face      */

/* ---- SWEPT: torque vectoring ---- */
extern float g_KP_YAW;             /* master TV yaw gain, Nm per rad/s             */
extern float g_TV_KI_FRAC;         /* integral gain as a fraction of KP_YAW        */
extern float g_TV_KD_FRAC;         /* derivative gain as a fraction of KP_YAW      */
extern float g_TV_KFF_FRAC;        /* feedforward gain as a fraction of KP_YAW     */
extern float g_TV_I_MAX_FRAC;      /* integral bias cap as a fraction of motor pk  */
extern float g_TV_SPEED_REF_MS;    /* neutral speed for the gain scaling, m/s      */
extern float g_TV_WHEEL_YAW_TRUST; /* weight on the wheel-speed yaw estimate (0-1)  */

/* ---- FIXED: not performance choices, not swept ---- */
extern float g_TV_YAW_DEADBAND; /* sensor-noise floor, rad/s                     */
extern float g_TV_K_US;         /* empirical understeer term (linear deriv ~0)   */

/* Apply any TUNE_<NAME> env overrides; call once at the top of main(). */
void tunables_init_from_env(void);

#endif /* TUNABLES_H */
