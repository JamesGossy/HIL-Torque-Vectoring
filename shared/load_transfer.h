#ifndef LOAD_TRANSFER_H
#define LOAD_TRANSFER_H

#include "vehicle_config.h"

/*
 * Per-wheel vertical load from static split, aero downforce and load transfer.
 * Shared so the vehicle model and the ECU grip estimate cannot drift apart.
 * Pass lagged accelerations (ax, ay) so the result does not ring at 100 Hz.
 */

// Fill Fz[FL,FR,RL,RR] in newtons. ax forward+, ay left+, v speed in m/s.
static inline void load_transfer(float v, float ax, float ay, float Fz[4])
{
    const float g = 9.81f;

    // static axle loads plus aero downforce, split 50/50 front/rear
    float q           = 0.5f * AIR_DENSITY * AERO_AREA * v * v;
    float F_downforce = CLA * q;
    float Fz_front    = MASS_KG * g * (CG_TO_REAR_M / WHEELBASE_M) + F_downforce * 0.5f;
    float Fz_rear     = MASS_KG * g * (CG_TO_FRONT_M / WHEELBASE_M) + F_downforce * 0.5f;

    // longitudinal transfer between axles
    float dFz_long = ax * MASS_KG * CG_HEIGHT_M / WHEELBASE_M;
    Fz_front -= dFz_long;
    Fz_rear += dFz_long;
    if (Fz_front < 50.0f) Fz_front = 50.0f;
    if (Fz_rear < 50.0f) Fz_rear = 50.0f;

    // lateral transfer at each axle, +ay loads the right wheels
    float m_front   = MASS_KG * (CG_TO_REAR_M / WHEELBASE_M);
    float m_rear    = MASS_KG * (CG_TO_FRONT_M / WHEELBASE_M);
    float dFz_lat_f = m_front * ay * CG_HEIGHT_M / TRACK_WIDTH_FRONT_M;
    float dFz_lat_r = m_rear * ay * CG_HEIGHT_M / TRACK_WIDTH_REAR_M;

    Fz[WHEEL_FL] = 0.5f * Fz_front - dFz_lat_f;
    Fz[WHEEL_FR] = 0.5f * Fz_front + dFz_lat_f;
    Fz[WHEEL_RL] = 0.5f * Fz_rear - dFz_lat_r;
    Fz[WHEEL_RR] = 0.5f * Fz_rear + dFz_lat_r;

    for (int i = 0; i < 4; i++)
        if (Fz[i] < 25.0f) Fz[i] = 25.0f;
}

#endif /* LOAD_TRANSFER_H */
