#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

class MotorControl {
public:
    MotorControl();
    
    // Initialize motor PWM outputs
    void begin();
    
    // Arm/disarm motors
    void arm();
    void disarm();
    bool isArmed() const { return armed; }
    
    // Set individual motor values (1000-2000 microseconds)
    void setMotor(uint8_t motor, uint16_t value);
    
    // Update motors with mixer (quadcopter X configuration)
    // throttle: base throttle (1000-2000)
    // roll, pitch, yaw: PID outputs (-500 to +500)
    void update(float throttle, float roll, float pitch, float yaw);
    
    // Stop all motors immediately
    void stop();
    
    // Get current motor values
    uint16_t getMotor(uint8_t motor) const;

private:
    bool armed;
    uint16_t motorValues[4];
    
    // Convert microseconds to PWM duty cycle
    uint32_t usToPWM(uint16_t us);
};

#endif // MOTOR_CONTROL_H
