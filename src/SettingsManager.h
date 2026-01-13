#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include "PIDController.h"
#include "IMU_Driver.h"
#include "HMC5883L.h"

class SettingsManager {
public:
    SettingsManager();
    
    // Initialize preferences
    void begin();
    
    // PID Settings
    void savePID(const char* axis, float kp, float ki, float kd, float imax);
    void loadPID(const char* axis, PIDController& pid);
    
    // IMU Calibration
    void saveIMUCalibration(const IMUDriver& imu);
    void loadIMUCalibration(IMUDriver& imu);
    
    // Magnetometer Calibration
    void saveMagCalibration(const HMC5883L& mag);
    void loadMagCalibration(HMC5883L& mag);
    
    // Global actions
    void saveAll(PIDController& roll, PIDController& pitch, PIDController& yaw);
    void resetToDefaults();

private:
    Preferences _prefs;
};

#endif // SETTINGS_MANAGER_H
