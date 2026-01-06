#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float imax)
    : kp(kp), ki(ki), kd(kd), imax(imax),
      outMin(-1000.0f), outMax(1000.0f),
      integral(0.0f), lastError(0.0f),
      lastP(0.0f), lastD(0.0f), firstRun(true) {}

void PIDController::setGains(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PIDController::compute(float error, float dt) {
    // Proportional term
    lastP = kp * error;
    
    // Integral term with anti-windup
    integral += ki * error * dt;
    integral = constrain(integral, -imax, imax);
    
    // Derivative term (on error)
    float derivative = 0.0f;
    if (!firstRun && dt > 0.0f) {
        derivative = (error - lastError) / dt;
    }
    lastD = kd * derivative;
    lastError = error;
    firstRun = false;
    
    // Calculate output
    float output = lastP + integral + lastD;
    return constrain(output, outMin, outMax);
}

float PIDController::compute(float error, float rate, float dt) {
    // Proportional term
    lastP = kp * error;
    
    // Integral term with anti-windup
    integral += ki * error * dt;
    integral = constrain(integral, -imax, imax);
    
    // Derivative term (on provided rate - avoids derivative kick)
    // Note: rate is typically the gyro reading (angular velocity)
    // We negate it because d(error)/dt = d(setpoint)/dt - d(measured)/dt
    // and setpoint is usually constant, so d(error)/dt = -rate
    lastD = -kd * rate;
    lastError = error;
    firstRun = false;
    
    // Calculate output
    float output = lastP + integral + lastD;
    return constrain(output, outMin, outMax);
}

void PIDController::reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastP = 0.0f;
    lastD = 0.0f;
    firstRun = true;
}
