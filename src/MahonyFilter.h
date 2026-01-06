#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include <Arduino.h>

class MahonyFilter {
public:
    MahonyFilter(float kp = 1.0f, float ki = 0.0f);
    
    // Reset filter to initial state
    void reset();
    
    // Update filter with new sensor data
    // gx, gy, gz: gyroscope in radians/sec
    // ax, ay, az: accelerometer (will be normalized internally)
    void update(float gx, float gy, float gz, 
                float ax, float ay, float az, 
                float dt);
    
    // Get quaternion components
    float getQ0() const { return q0; }
    float getQ1() const { return q1; }
    float getQ2() const { return q2; }
    float getQ3() const { return q3; }
    
    // Get Euler angles in degrees
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    
    // Get Euler angles in radians
    float getRollRad() const;
    float getPitchRad() const;
    float getYawRad() const;
    
    // Tuning parameters
    void setKp(float kp) { twoKp = 2.0f * kp; }
    void setKi(float ki) { twoKi = 2.0f * ki; }

private:
    // Quaternion of sensor frame relative to earth frame
    float q0, q1, q2, q3;
    
    // Integral error terms (for Ki)
    float integralFBx, integralFBy, integralFBz;
    
    // Filter gains (stored as 2*Kp and 2*Ki for efficiency)
    float twoKp;
    float twoKi;
    
    // Fast inverse square root (Quake III algorithm)
    static float invSqrt(float x);
};

#endif // MAHONY_FILTER_H
