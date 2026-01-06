#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>
#include "config.h"

class IMUDriver {
public:
    // Initialize the MPU6050
    bool begin();
    
    // Read all sensor data
    void update();
    
    // Calibrate gyro offsets (call when drone is stationary)
    void calibrate(int samples = 1000);
    
    // Get scaled accelerometer values (in g)
    float getAccelX() const { return (accelX - accelOffsetX) / 16384.0f; }
    float getAccelY() const { return (accelY - accelOffsetY) / 16384.0f; }
    float getAccelZ() const { return (accelZ - accelOffsetZ) / 16384.0f; }
    
    // Get scaled gyroscope values (in degrees/sec)
    float getGyroX() const { return (gyroX - gyroOffsetX) / 131.0f; }
    float getGyroY() const { return (gyroY - gyroOffsetY) / 131.0f; }
    float getGyroZ() const { return (gyroZ - gyroOffsetZ) / 131.0f; }
    
    // Get raw values
    int16_t getRawAccelX() const { return accelX; }
    int16_t getRawAccelY() const { return accelY; }
    int16_t getRawAccelZ() const { return accelZ; }
    int16_t getRawGyroX() const { return gyroX; }
    int16_t getRawGyroY() const { return gyroY; }
    int16_t getRawGyroZ() const { return gyroZ; }
    int16_t getTemperature() const { return temperature; }
    
    // Calibration offsets
    int16_t gyroOffsetX = 0;
    int16_t gyroOffsetY = 0;
    int16_t gyroOffsetZ = 0;
    int16_t accelOffsetX = 0;
    int16_t accelOffsetY = 0;
    int16_t accelOffsetZ = 0;

private:
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temperature;
};

#endif // IMU_DRIVER_H
