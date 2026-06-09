#include "../include/lqr_steer.h"
#include "../../shared/vehicle_config.h"
#include "../../shared/parameters_config.h"
#include <math.h>

/*
 * Stage-1 LQR steering on the dynamic-bicycle lateral error dynamics.
 *
 * Continuous error model (Rajamani, "Vehicle Dynamics and Control", ch.3), state
 * x = [e1, e1_dot, e2, e2_dot]^T, input u = steer angle delta:
 *
 *   d/dt e1     = e1_dot
 *   d/dt e1_dot = -(2Cf+2Cr)/(m vx) e1_dot
 *                 + (2Cf+2Cr)/m e2
 *                 + (-2Cf lf + 2Cr lr)/(m vx) e2_dot
 *                 + (2Cf/m) delta            (+ curvature feedforward terms)
 *   d/dt e2     = e2_dot
 *   d/dt e2_dot = -(2Cf lf - 2Cr lr)/(Iz vx) e1_dot
 *                 + (2Cf lf - 2Cr lr)/Iz e2
 *                 - (2Cf lf^2 + 2Cr lr^2)/(Iz vx) e2_dot
 *                 + (2Cf lf/Iz) delta
 *
 * The matrices depend on vx, so the gain is recomputed when vx changes by more
 * than a threshold. The infinite-horizon discrete LQR gain is found by iterating
 * the Riccati recursion to convergence (cheap: 4x4, done rarely).
 *
 * Cornering stiffness is the Pacejka slope at zero slip, C_alpha = D*Fz*|C|*B,
 * per axle at static load. The plant has load transfer and combined slip the
 * model ignores; the feedback law is robust to that mismatch.
 */

/* Nominal Ackermann ratio: reference_angle * ACK_NOMINAL = road-wheel angle.
 * The model uses inner/outer ratios ~0.20-0.26; 0.23 is their midpoint. */
#define ACK_NOMINAL 0.23f

/* Per-axle cornering stiffness, N/rad (computed once from config at first call) */
static float Cf, Cr;
static int stiff_ready = 0;

static void init_stiffness(void)
{
    const float g = 9.81f;
    float Wf      = MASS_KG * g * CG_TO_REAR_M / WHEELBASE_M; /* front static load */
    float Wr      = MASS_KG * g * CG_TO_FRONT_M / WHEELBASE_M;
    /* C_alpha per axle = D*Fz*|C|*B (Pacejka slope at alpha=0). */
    Cf          = TYRE_D * Wf * fabsf(TYRE_C) * TYRE_B;
    Cr          = TYRE_D * Wr * fabsf(TYRE_C) * TYRE_B;
    stiff_ready = 1;
}

/* ---- tiny 4x4 linear algebra for the Riccati iteration ---- */
#define N 4
typedef float Mat[N][N];
typedef float Vec[N];

static void mat_mul(const Mat a, const Mat b, Mat out)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++)
                s += a[i][k] * b[k][j];
            out[i][j] = s;
        }
}

static void mat_T(const Mat a, Mat out)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            out[i][j] = a[j][i];
}

/*
 * Discretise the continuous (Ac, Bc) by forward Euler over CONTROL_DT_S:
 *   Ad = I + Ac*dt,  Bd = Bc*dt.
 * Euler is adequate at 100 Hz for this well-damped plant and keeps the code
 * simple; the gain is a feedback law, not an open-loop predictor, so small
 * discretisation error is corrected online.
 */
static void discretise(const Mat Ac, const Vec Bc, Mat Ad, Vec Bd)
{
    const float dt = CONTROL_DT_S;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++)
            Ad[i][j] = (i == j ? 1.0f : 0.0f) + Ac[i][j] * dt;
        Bd[i] = Bc[i] * dt;
    }
}

/*
 * Infinite-horizon discrete LQR gain K for (Ad, Bd) with diagonal Q and scalar
 * R, by iterating the Riccati recursion to convergence:
 *   P <- Q + Ad^T P Ad - Ad^T P Bd (R + Bd^T P Bd)^-1 Bd^T P Ad
 *   K  = (R + Bd^T P Bd)^-1 Bd^T P Ad
 * Single-input, so the inverse is a scalar reciprocal.
 */
static void lqr_gain(const Mat Ad, const Vec Bd, const Vec Qdiag, float R, Vec K)
{
    Mat P;
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            P[i][j] = (i == j) ? Qdiag[i] : 0.0f;

    Mat AdT;
    mat_T(Ad, AdT);

    for (int iter = 0; iter < 200; iter++) {
        /* PA = P*Ad ; AtP = Ad^T*P */
        Mat PA, AtP;
        mat_mul(P, Ad, PA);
        mat_mul(AdT, P, AtP);

        /* Bd^T P Bd  (scalar) and Bd^T P Ad (row vector) */
        float BtPB = 0.0f;
        Vec BtPA;
        for (int j = 0; j < N; j++) {
            float pb = 0.0f, pa_col = 0.0f;
            for (int k = 0; k < N; k++)
                pb += P[j][k] * Bd[k]; /* (P Bd)_j */
            (void)pa_col;
            BtPA[j] = 0.0f; /* fill below */
            BtPB += Bd[j] * pb;
        }
        /* BtPA_j = sum_k Bd_k (P Ad)_{k j} = sum_k Bd_k PA[k][j] */
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++)
                s += Bd[k] * PA[k][j];
            BtPA[j] = s;
        }

        float inv = 1.0f / (R + BtPB);
        Vec Kn;
        for (int j = 0; j < N; j++)
            Kn[j] = inv * BtPA[j];

        /* P_next = Q + Ad^T P Ad - (Ad^T P Bd) * Kn
         *        = Q + AtP*Ad - (AtP*Bd) outer Kn                            */
        Mat AtPA;
        mat_mul(AtP, Ad, AtPA);
        Vec AtPB;
        for (int i = 0; i < N; i++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++)
                s += AtP[i][k] * Bd[k];
            AtPB[i] = s;
        }

        Mat Pn;
        float maxdiff = 0.0f;
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) {
                float q  = (i == j) ? Qdiag[i] : 0.0f;
                Pn[i][j] = q + AtPA[i][j] - AtPB[i] * Kn[j];
                float d  = fabsf(Pn[i][j] - P[i][j]);
                if (d > maxdiff) maxdiff = d;
            }

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                P[i][j] = Pn[i][j];

        for (int j = 0; j < N; j++)
            K[j] = Kn[j];
        if (maxdiff < 1e-3f) break;
    }
}

/* ---- cached gain, recomputed when speed moves enough ---- */
static Vec cached_K;
static float cached_vx = -1.0f;

/* Integral of the cross-track error. A pure LQR on a curved path holds a small
 * STEADY-STATE e1 (it trades cross-track error for the extra steer the kinematic
 * feedforward under-provides once sideslip is accounted for). On a sustained
 * corner that ~0.15-0.2 m offset is enough to eat the racing line's thin apex
 * margin and clip a cone. Integral action drives that steady-state e1 to zero.
 * Anti-windup: clamped, and it only integrates when e1 is small enough that we
 * are tracking (not during a big transient). */
static float e1_integral = 0.0f;

#ifndef LQR_KI
#define LQR_KI 2.0000f /* integral gain on e1, reference-rad per (m*s)   */
#endif
#ifndef LQR_I_MAX
#define LQR_I_MAX 0.3318f /* clamp on the integral's steering contribution  */
#endif

/* Reset the integrator between independent runs / test cases. */
void lqr_steer_reset(void)
{
    e1_integral = 0.0f;
    cached_vx   = -1.0f;
}

/* Cost weights (the "tuning" of the controller). Q penalises [e1, e1_dot, e2,
 * e2_dot]; R penalises steering effort. e1 (cross-track) is weighted hard so the
 * car holds the line through the apex; modest e2 keeps the heading aligned. */
#ifndef LQR_Q_E1
#define LQR_Q_E1 30.2839f
#endif
#ifndef LQR_Q_E1D
#define LQR_Q_E1D 0.8083f
#endif
#ifndef LQR_Q_E2
#define LQR_Q_E2 11.5161f
#endif
#ifndef LQR_Q_E2D
#define LQR_Q_E2D 1.0000f
#endif
#ifndef LQR_R
#define LQR_R 5.9415f
#endif

/*
 * Build the discrete error-dynamics model (Ad, Bd) at speed vx. The input is the
 * steering REFERENCE (Ackermann ratio folded into B).
 */
static void build_model(float vx, Mat Ad, Vec Bd)
{
    if (!stiff_ready) init_stiffness();
    if (vx < 1.0f) vx = 1.0f; /* model singular at vx=0 */

    float m = MASS_KG, Iz = YAW_INERTIA_KGM2;
    float lf = CG_TO_FRONT_M, lr = CG_TO_REAR_M;

    /* Continuous error-dynamics matrix Ac and input Bc (Rajamani form). */
    Mat Ac = { { 0 } };
    Vec Bc = { 0 };

    Ac[0][1] = 1.0f;
    Ac[1][1] = -(2.0f * Cf + 2.0f * Cr) / (m * vx);
    Ac[1][2] = (2.0f * Cf + 2.0f * Cr) / m;
    Ac[1][3] = (-2.0f * Cf * lf + 2.0f * Cr * lr) / (m * vx);
    Ac[2][3] = 1.0f;
    Ac[3][1] = -(2.0f * Cf * lf - 2.0f * Cr * lr) / (Iz * vx);
    Ac[3][2] = (2.0f * Cf * lf - 2.0f * Cr * lr) / Iz;
    Ac[3][3] = -(2.0f * Cf * lf * lf + 2.0f * Cr * lr * lr) / (Iz * vx);

    /* The control input is the STEERING REFERENCE, not the physical road-wheel
     * angle: the vehicle model multiplies the reference by the Ackermann ratio
     * (~0.23) to get the wheel angle. Fold that into B so the gain is designed in
     * reference units directly - otherwise converting the output to a reference
     * afterwards inflates the feedback by 1/0.23 (~4.3x) and the loop oscillates. */
    Bc[1] = (2.0f * Cf / m) * ACK_NOMINAL;
    Bc[3] = (2.0f * Cf * lf / Iz) * ACK_NOMINAL;

    discretise(Ac, Bc, Ad, Bd);
}

static void recompute_gain(float vx)
{
    Mat Ad;
    Vec Bd;
    build_model(vx, Ad, Bd);

    Vec Q = { LQR_Q_E1, LQR_Q_E1D, LQR_Q_E2, LQR_Q_E2D };
    lqr_gain(Ad, Bd, Q, LQR_R, cached_K);
    cached_vx = vx;
}

float lqr_steer_command(float vx, float vy, float e1, float e2, float yaw_rate, float path_kappa)
{
    if (cached_vx < 0.0f || fabsf(vx - cached_vx) > 1.0f) recompute_gain(vx);

    if (vx < 1.0f) vx = 1.0f;

    /* Error-rate states from measurements (Rajamani):
     *   e1_dot = vy + vx*e2   - the true cross-track rate includes body sideslip
     *            vy, not just the heading-error closing term. Dropping vy was the
     *            bug that let the car drift on fast sections where vy is large.
     *   e2_dot = yaw_rate - vx*path_kappa  (the path's own yaw rate is vx*kappa). */
    float e2_dot = yaw_rate - vx * path_kappa;
    float e1_dot = vy + vx * e2;

    /* LQR feedback, already in REFERENCE units (the Ackermann ratio is folded
     * into B), so do not rescale it: delta_fb = -K x. */
    float delta_fb
        = -(cached_K[0] * e1 + cached_K[1] * e1_dot + cached_K[2] * e2 + cached_K[3] * e2_dot);

    /* Curvature feedforward: the steady-state ROAD-WHEEL steer to hold radius
     * 1/kappa, incl. the understeer term: delta_ff = L*kappa + Kus*vx^2*kappa.
     * Convert to a reference by dividing by the Ackermann ratio. */
    float m = MASS_KG, lf = CG_TO_FRONT_M, lr = CG_TO_REAR_M;
    float Kus            = (m / (2.0f * WHEELBASE_M)) * (lr / Cf - lf / Cr); /* understeer grad */
    float delta_ff_wheel = WHEELBASE_M * path_kappa + Kus * vx * vx * path_kappa;
    float delta_ff       = delta_ff_wheel / ACK_NOMINAL;
    (void)lf;

    /* Integral action on e1 to remove the steady-state cross-track offset. Only
     * integrate while reasonably on-line (|e1| < 1 m) so a big transient does not
     * wind it up, and clamp the contribution. */
    if (fabsf(e1) < 1.0f) {
        e1_integral += e1 * CONTROL_DT_S;
        float i_lim = LQR_I_MAX / LQR_KI;
        if (e1_integral > i_lim) e1_integral = i_lim;
        if (e1_integral < -i_lim) e1_integral = -i_lim;
    }
    /* e1 = -cte, so +e1 (car left of line) needs steer to the right; the LQR
     * feedback already uses this sign via -K, so the integral follows it: the
     * correction is -LQR_KI * integral(e1). */
    float delta_i = -LQR_KI * e1_integral;

    return delta_fb + delta_ff + delta_i;
}
