#include "MahonyFilter.h"
#include <math.h>

MahonyFilter::MahonyFilter(float kp, float ki) 
    : twoKp(2.0f * kp), twoKi(2.0f * ki) {
    reset();
}

void MahonyFilter::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void MahonyFilter::update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Apply integral feedback if Ki > 0
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = q0;
    qb = q1;
    qc = q2;
    
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float MahonyFilter::getRollRad() const {
    return atan2f(2.0f * (q0 * q1 + q2 * q3), 
                  1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

float MahonyFilter::getPitchRad() const {
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        return copysignf(M_PI / 2.0f, sinp);
    }
    return asinf(sinp);
}

float MahonyFilter::getYawRad() const {
    return atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

float MahonyFilter::getRoll() const {
    return getRollRad() * 180.0f / M_PI;
}

float MahonyFilter::getPitch() const {
    return getPitchRad() * 180.0f / M_PI;
}

float MahonyFilter::getYaw() const {
    return getYawRad() * 180.0f / M_PI;
}

// Fast inverse square root (Quake III algorithm)
// More accurate than 1.0f/sqrtf(x) on ESP32 and faster
float MahonyFilter::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));  // Newton iteration
    y = y * (1.5f - (halfx * y * y));  // Second iteration for accuracy
    return y;
}
