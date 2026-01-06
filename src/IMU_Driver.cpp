#include "IMU_Driver.h"
#include <Wire.h>

#define MPU6050_ADDR    0x68
#define PWR_MGMT_1      0x6B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define CONFIG_REG      0x1A
#define ACCEL_XOUT_H    0x3B

bool IMUDriver::begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK);
    
    // Wake up the MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    delay(100);  // Wait for sensor to stabilize
    
    // Configure gyroscope (±250°/s for best resolution)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x00);  // FS_SEL = 0 -> ±250°/s
    Wire.endTransmission();
    
    // Configure accelerometer (±2g for best resolution)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x00);  // AFS_SEL = 0 -> ±2g
    Wire.endTransmission();
    
    // Configure digital low-pass filter (DLPF)
    // Setting 3: Accel 44Hz, Gyro 42Hz bandwidth
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(CONFIG_REG);
    Wire.write(0x03);
    Wire.endTransmission();
    
    return true;
}

void IMUDriver::update() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

    if (Wire.available() == 14) {
        accelX = (int16_t)(Wire.read() << 8 | Wire.read());
        accelY = (int16_t)(Wire.read() << 8 | Wire.read());
        accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
        temperature = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
        gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
    }
}

void IMUDriver::calibrate(int samples) {
    long gxSum = 0, gySum = 0, gzSum = 0;
    long axSum = 0, aySum = 0, azSum = 0;
    
    Serial.println("Calibrating IMU... Keep the drone STILL and LEVEL!");
    delay(2000);
    
    for (int i = 0; i < samples; i++) {
        update();
        gxSum += gyroX;
        gySum += gyroY;
        gzSum += gyroZ;
        axSum += accelX;
        aySum += accelY;
        azSum += accelZ;
        
        if (i % 100 == 0) {
            Serial.printf("Calibration: %d/%d\n", i, samples);
        }
        delay(2);  // ~500Hz sampling
    }
    
    gyroOffsetX = gxSum / samples;
    gyroOffsetY = gySum / samples;
    gyroOffsetZ = gzSum / samples;
    
    accelOffsetX = axSum / samples;
    accelOffsetY = aySum / samples;
    // Z-axis should read 1g (16384 for ±2g range), so offset accordingly
    accelOffsetZ = (azSum / samples) - 16384;
    
    Serial.println("Calibration complete!");
    Serial.printf("Gyro offsets: X=%d, Y=%d, Z=%d\n", 
                  gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    Serial.printf("Accel offsets: X=%d, Y=%d, Z=%d\n", 
                  accelOffsetX, accelOffsetY, accelOffsetZ);
}
