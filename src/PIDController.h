#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float imax = 100.0f);
    
    // Set PID gains
    void setGains(float kp, float ki, float kd);
    void setKp(float kp) { this->kp = kp; }
    void setKi(float ki) { this->ki = ki; }
    void setKd(float kd) { this->kd = kd; }
    
    // Set integral windup limit
    void setIntegralMax(float imax) { this->imax = imax; }
    
    // Set output limits
    void setOutputLimits(float min, float max) { outMin = min; outMax = max; }
    
    // Compute PID output
    // error: setpoint - measured value
    // dt: time step in seconds
    float compute(float error, float dt);
    
    // Compute PID with rate (uses provided rate instead of calculating from error)
    // Useful for avoiding derivative kick
    float compute(float error, float rate, float dt);
    
    // Reset integral and derivative state
    void reset();
    
    // Get individual PID components (for tuning)
    float getP() const { return lastP; }
    float getI() const { return integral; }
    float getD() const { return lastD; }
    
private:
    float kp, ki, kd;
    float imax;
    float outMin, outMax;
    float integral;
    float lastError;
    float lastP, lastD;
    bool firstRun;
};

#endif // PID_CONTROLLER_H
