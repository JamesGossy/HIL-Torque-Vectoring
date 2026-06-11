#ifndef TUNABLES_H
#define TUNABLES_H

/* Externs for every controller gain. Swept gains take a TUNE_<NAME> env override. See tunables.c for values. */

/* ---- steering ---- */
extern float g_GRIP_USE;            /* fraction of physical peak lateral grip       */
extern float g_K_STANLEY;           /* Stanley cross-track gain                     */
extern float g_K_DAMP;              /* Stanley yaw-rate damping gain                */
extern float g_RACING_MARGIN;       /* racing-line corridor safety margin           */
extern float g_PP_RADIUS_FACTOR;    /* opens the racing-line kinematic radius floor */
extern float g_MAX_STEER_RAD;       /* steering reference limit (driver cap)        */
extern float g_MAX_STEER_RATE_RADS; /* steering slew limit, rad/s                   */
extern float g_STEER_SAT_FRAC;      /* lock fraction above which throttle fades     */

/* ---- speed planner ---- */
extern float g_SPEED_PLAN_HORIZON_M; /* corner-scan lookahead, m                    */
extern int g_SPEED_PLAN_STEPS;       /* scan depth (clamped to SPEED_PLAN_STEPS_CAP) */
extern float g_MAX_BRAKE_DECEL_MS2;  /* braking-effort cap, m/s^2                    */
extern float g_SPEED_KP_NM;          /* throttle P-gain, Nm per m/s                  */
extern float g_BRAKE_KP_NM;          /* brake P-gain, Nm per m/s                     */
extern float g_SPEED_KI_FRAC;        /* throttle I-gain as fraction of KP_NM (Ki = frac * Kp) */
extern float g_SPEED_KI_NM;          /* derived: g_SPEED_KI_FRAC * g_SPEED_KP_NM    */
extern float g_SPEED_I_MAX_NM;       /* throttle integral clamp, Nm                  */
extern int g_NEAREST_SEARCH_BACK;    /* segments searched back (clamped to cap)      */
extern int g_NEAREST_SEARCH_FWD;     /* segments searched fwd (clamped to cap)       */

/* ---- torque vectoring ---- */
extern float g_KP_YAW;             /* master TV yaw gain, Nm per rad/s             */
extern float g_TV_KI_FRAC;         /* integral gain as a fraction of KP_YAW        */
extern float g_TV_KD_FRAC;         /* derivative gain as a fraction of KP_YAW      */
extern float g_TV_KFF_FRAC;        /* feedforward gain as a fraction of KP_YAW     */
extern float g_TV_I_MAX_FRAC;      /* integral bias cap as a fraction of motor pk  */

extern float g_TV_YAW_DEADBAND; /* sensor-noise floor, rad/s                     */
extern float g_TV_K_US;         /* empirical understeer term (linear deriv ~0)   */

/* ---- autonomy: cone sensor ---- */
extern float g_SENSOR_RANGE_M;         /* max detection range, m                       */
extern float g_SENSOR_FOV_RAD;         /* half field of view, rad                      */
extern float g_SENSOR_RANGE_SIGMA_M;   /* range noise std, m (also EKF R)              */
extern float g_SENSOR_BEARING_SIGMA_RAD; /* bearing noise std, rad (also EKF R)        */

/* ---- autonomy: EKF-SLAM ---- */
extern float g_SLAM_Q_POS;          /* process noise on position per tick, m^2      */
extern float g_SLAM_Q_THETA;        /* process noise on heading per tick, rad^2     */
extern float g_SLAM_GATE_CHI2;      /* Mahalanobis gate to associate (2-DoF)        */
extern float g_SLAM_NEW_CHI2;       /* Mahalanobis floor to allow a new landmark    */
extern float g_SLAM_NEW_MIN_DIST_M; /* Euclidean guard for a new landmark, m        */
extern int g_SLAM_MIN_SIGHTINGS;    /* sightings before a landmark enters the map   */
extern float g_SLAM_LOOP_RADIUS_M;  /* return-to-start radius for loop closure, m   */
extern float g_SLAM_AMBIG_RATIO;    /* runner-up/best d2 ratio to accept a match    */

/* ---- autonomy: mode and planner ---- */
extern int g_AUTONOMY;             /* 0 = legacy ground-truth driver, 1 = SLAM     */
extern float g_PHASE1_SPEED_CAP_MS; /* exploration-lap speed cap, m/s              */

/* Apply any TUNE_<NAME> env overrides; call once at the top of main(). */
void tunables_init_from_env(void);

#endif /* TUNABLES_H */
