#include "MotorControl.h"

MotorControl::MotorControl() : armed(false) {
    for (int i = 0; i < 4; i++) {
        motorValues[i] = PWM_MIN;
    }
}

void MotorControl::begin() {
    // Configure LEDC channels for each motor
    // ESP32 LEDC provides precise PWM output
    
    // Motor 0 - Front Left
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PIN_FL, 0);
    
    // Motor 1 - Front Right
    ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PIN_FR, 1);
    
    // Motor 2 - Back Left
    ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PIN_BL, 2);
    
    // Motor 3 - Back Right
    ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PIN_BR, 3);
    
    // Initialize all motors to minimum
    stop();
}

void MotorControl::arm() {
    armed = true;
    // Send minimum throttle to arm ESCs
    for (int i = 0; i < 4; i++) {
        motorValues[i] = THROTTLE_IDLE;
        ledcWrite(i, usToPWM(motorValues[i]));
    }
}

void MotorControl::disarm() {
    armed = false;
    stop();
}

void MotorControl::stop() {
    for (int i = 0; i < 4; i++) {
        motorValues[i] = PWM_MIN;
        ledcWrite(i, usToPWM(PWM_MIN));
    }
}

void MotorControl::setMotor(uint8_t motor, uint16_t value) {
    if (motor >= 4) return;
    
    value = constrain(value, PWM_MIN, PWM_MAX);
    motorValues[motor] = value;
    
    if (armed) {
        ledcWrite(motor, usToPWM(value));
    }
}

void MotorControl::update(float throttle, float roll, float pitch, float yaw) {
    if (!armed) {
        stop();
        return;
    }
    
    // Quadcopter X configuration motor mixing:
    //
    //     Front
    //   FL     FR
    //     \   /
    //      \ /
    //       X
    //      / \
    //     /   \
    //   BL     BR
    //     Back
    //
    // FL (0): +throttle -roll +pitch -yaw (CCW)
    // FR (1): +throttle +roll +pitch +yaw (CW)
    // BL (2): +throttle -roll -pitch +yaw (CW)
    // BR (3): +throttle +roll -pitch -yaw (CCW)
    
    float motors[4];
    motors[0] = throttle - roll + pitch - yaw;  // Front Left
    motors[1] = throttle + roll + pitch + yaw;  // Front Right
    motors[2] = throttle - roll - pitch + yaw;  // Back Left
    motors[3] = throttle + roll - pitch - yaw;  // Back Right
    
    // Find the maximum motor value for scaling
    float maxMotor = motors[0];
    float minMotor = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxMotor) maxMotor = motors[i];
        if (motors[i] < minMotor) minMotor = motors[i];
    }
    
    // Scale motors if any exceed limits (maintain relative differences)
    if (maxMotor > THROTTLE_MAX) {
        float scale = (THROTTLE_MAX - throttle) / (maxMotor - throttle);
        for (int i = 0; i < 4; i++) {
            motors[i] = throttle + (motors[i] - throttle) * scale;
        }
    }
    
    // Apply motor values
    for (int i = 0; i < 4; i++) {
        motorValues[i] = constrain((uint16_t)motors[i], THROTTLE_IDLE, THROTTLE_MAX);
        ledcWrite(i, usToPWM(motorValues[i]));
    }
}

uint16_t MotorControl::getMotor(uint8_t motor) const {
    if (motor >= 4) return 0;
    return motorValues[motor];
}

uint32_t MotorControl::usToPWM(uint16_t us) {
    // Convert microseconds to LEDC duty cycle
    // At 250Hz (4ms period) with 12-bit resolution (0-4095):
    // 1000us = 1024 (25% duty)
    // 2000us = 2048 (50% duty)
    
    // Calculate duty cycle based on PWM frequency and resolution
    // duty = (us / period_us) * max_duty
    float periodUs = 1000000.0f / PWM_FREQUENCY;
    uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
    return (uint32_t)((us / periodUs) * maxDuty);
}
