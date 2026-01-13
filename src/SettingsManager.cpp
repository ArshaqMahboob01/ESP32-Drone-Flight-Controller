#include "SettingsManager.h"

SettingsManager::SettingsManager() {}

void SettingsManager::begin() {
    _prefs.begin("drone", false);
}

void SettingsManager::savePID(const char* axis, float kp, float ki, float kd, float imax) {
    char key[16];
    snprintf(key, sizeof(key), "%s_kp", axis);
    _prefs.putFloat(key, kp);
    snprintf(key, sizeof(key), "%s_ki", axis);
    _prefs.putFloat(key, ki);
    snprintf(key, sizeof(key), "%s_kd", axis);
    _prefs.putFloat(key, kd);
    snprintf(key, sizeof(key), "%s_imax", axis);
    _prefs.putFloat(key, imax);
}

void SettingsManager::loadPID(const char* axis, PIDController& pid) {
    char key[16];
    snprintf(key, sizeof(key), "%s_kp", axis);
    float kp = _prefs.getFloat(key, -1.0f);
    if (kp < 0) return; // Not set
    
    snprintf(key, sizeof(key), "%s_ki", axis);
    float ki = _prefs.getFloat(key, 0.0f);
    snprintf(key, sizeof(key), "%s_kd", axis);
    float kd = _prefs.getFloat(key, 0.0f);
    snprintf(key, sizeof(key), "%s_imax", axis);
    float imax = _prefs.getFloat(key, 50.0f);
    
    pid.setGains(kp, ki, kd);
    pid.setIntegralMax(imax);
}

void SettingsManager::saveIMUCalibration(const IMUDriver& imu) {
    _prefs.putShort("gx_off", imu.gyroOffsetX);
    _prefs.putShort("gy_off", imu.gyroOffsetY);
    _prefs.putShort("gz_off", imu.gyroOffsetZ);
    _prefs.putShort("ax_off", imu.accelOffsetX);
    _prefs.putShort("ay_off", imu.accelOffsetY);
    _prefs.putShort("az_off", imu.accelOffsetZ);
}

void SettingsManager::loadIMUCalibration(IMUDriver& imu) {
    if (!_prefs.isKey("gx_off")) return;
    
    imu.gyroOffsetX = _prefs.getShort("gx_off", 0);
    imu.gyroOffsetY = _prefs.getShort("gy_off", 0);
    imu.gyroOffsetZ = _prefs.getShort("gz_off", 0);
    imu.accelOffsetX = _prefs.getShort("ax_off", 0);
    imu.accelOffsetY = _prefs.getShort("ay_off", 16384); // Default to 1g if unset? No, 0 for offset
    imu.accelOffsetZ = _prefs.getShort("az_off", 0);
}

void SettingsManager::saveMagCalibration(const HMC5883L& mag) {
    _prefs.putShort("mx_off", mag.offsetX);
    _prefs.putShort("my_off", mag.offsetY);
    _prefs.putShort("mz_off", mag.offsetZ);
    _prefs.putFloat("mx_sc", mag.scaleX);
    _prefs.putFloat("my_sc", mag.scaleY);
    _prefs.putFloat("mz_sc", mag.scaleZ);
}

void SettingsManager::loadMagCalibration(HMC5883L& mag) {
    if (!_prefs.isKey("mx_off")) return;
    
    mag.offsetX = _prefs.getShort("mx_off", 0);
    mag.offsetY = _prefs.getShort("my_off", 0);
    mag.offsetZ = _prefs.getShort("mz_off", 0);
    mag.scaleX = _prefs.getFloat("mx_sc", 1.0f);
    mag.scaleY = _prefs.getFloat("my_sc", 1.0f);
    mag.scaleZ = _prefs.getFloat("mz_sc", 1.0f);
}

void SettingsManager::saveAll(PIDController& roll, PIDController& pitch, PIDController& yaw) {
    savePID("roll", roll.getP(), roll.getI(), roll.getD(), 50.0f); // Default imax for now
    savePID("pitch", pitch.getP(), pitch.getI(), pitch.getD(), 50.0f);
    savePID("yaw", yaw.getP(), yaw.getI(), yaw.getD(), 50.0f);
}

void SettingsManager::resetToDefaults() {
    _prefs.clear();
}
