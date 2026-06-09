#ifndef LQR_STEER_H
#define LQR_STEER_H

/*
 * lqr_steer.h  --  model-based steering law (Stage-1 toward MPC).
 *
 * The project's steering controller: an optimal feedback law on the
 * DYNAMIC-bicycle lateral error dynamics. It is also the foundation stage of an
 * MPC build - it uses the same prediction model and cost an MPC would, but solves
 * the unconstrained infinite-horizon problem (LQR) in closed form instead of a
 * per-tick constrained QP, so the QP/horizon machinery can later be layered on
 * the same model and cost.
 *
 * Error state (Rajamani lateral-control form), relative to the path:
 *   e1      cross-track error, m
 *   e1_dot  its rate, m/s
 *   e2      heading error (yaw - path heading), rad
 *   e2_dot  its rate, rad/s
 *
 * The plant is the real 4-corner model; this controller predicts with a
 * linearized single-track model (per-axle cornering stiffness from the Pacejka
 * slope). The gain is speed-dependent, so it is recomputed when speed moves
 * enough - the "linear time-varying" part.
 *
 * Returns the commanded front-axle steering reference (rad); the vehicle model
 * scales it by the Ackermann ratios to get the road-wheel angle.
 *
 * The tracker holds the line tightly (mean cross-track error ~0.09 m and it does
 * not run wide at apexes), which lets the speed planner carry nearly the full
 * grip budget for a clean ~26.5 s lap. Because it tracks so tightly it loads the
 * front hard at the apex, so it needs the racing line shaped to match: enough
 * apex clearance (RACING_MARGIN) and an opened hairpin radius (PP_MIN_RADIUS_M),
 * both in path_planning.c. The line, not the controller, is the limiter at the
 * hairpin. The tuning gains (Q/R cost weights, LQR_KI/LQR_I_MAX) are the
 * #ifndef-wrapped defaults below in lqr_steer.c; all came from the
 * robustness-aware sweep (tools/smart_sweep_lqr.py). No -D overrides are needed.
 */

/* Reset internal state (the cross-track integrator and the cached gain).
 * Call between independent runs or test cases. */
void lqr_steer_reset(void);

float lqr_steer_command(float vx,          /* longitudinal speed, m/s        */
                        float vy,          /* lateral (sideslip) velocity, m/s*/
                        float e1,          /* cross-track error, m           */
                        float e2,          /* heading error, rad             */
                        float yaw_rate,    /* measured yaw rate, rad/s       */
                        float path_kappa); /* path curvature at the car, 1/m */

#endif /* LQR_STEER_H */
