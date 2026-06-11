#ifndef LINALG_H
#define LINALG_H

/*
 * Tiny dense linear algebra for the EKF-SLAM filter. Header-only, plain float
 * arrays, row-major. Only the operations the filter needs: small fixed-shape
 * matmuls and a closed-form 2x2 inverse (the innovation covariance is always
 * 2x2 for a single range/bearing observation). No generic NxN solver, on
 * purpose: the EKF sparsity means everything else is a small block product.
 */

#include <math.h>

static const float LINALG_PI = 3.14159265358979323846f;

// Wrap an angle to (-pi, pi].
static inline float wrap_angle(float a)
{
    while (a > LINALG_PI)
        a -= 2.0f * LINALG_PI;
    while (a <= -LINALG_PI)
        a += 2.0f * LINALG_PI;
    return a;
}

// C(ar x bc) = A(ar x ac) * B(ac x bc), row-major.
static inline void mat_mul(
    const float *A, int ar, int ac, const float *B, int bc, float *C)
{
    for (int i = 0; i < ar; i++) {
        for (int j = 0; j < bc; j++) {
            float s = 0.0f;
            for (int k = 0; k < ac; k++)
                s += A[i * ac + k] * B[k * bc + j];
            C[i * bc + j] = s;
        }
    }
}

// C(ar x br) = A(ar x ac) * B^T, where B is (br x ac), row-major. Used for P*H^T.
static inline void mat_mul_T(
    const float *A, int ar, int ac, const float *B, int br, float *C)
{
    for (int i = 0; i < ar; i++) {
        for (int j = 0; j < br; j++) {
            float s = 0.0f;
            for (int k = 0; k < ac; k++)
                s += A[i * ac + k] * B[j * ac + k];
            C[i * br + j] = s;
        }
    }
}

// Closed-form inverse of a 2x2 (row-major: [a b; c d]). Returns 0 if singular.
static inline int inv2x2(const float S[4], float Sinv[4])
{
    float det = S[0] * S[3] - S[1] * S[2];
    if (fabsf(det) < 1e-12f) return 0;
    float idet = 1.0f / det;
    Sinv[0]    = S[3] * idet;
    Sinv[1]    = -S[1] * idet;
    Sinv[2]    = -S[2] * idet;
    Sinv[3]    = S[0] * idet;
    return 1;
}

#endif /* LINALG_H */
