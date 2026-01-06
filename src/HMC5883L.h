#ifndef HMC5883L_H
#define HMC5883L_H

#include <Arduino.h>
#include <Wire.h>

// HMC5883L I2C address
#define HMC5883L_ADDR       0x1E

// HMC5883L registers
#define HMC5883L_CONFIG_A   0x00
#define HMC5883L_CONFIG_B   0x01
#define HMC5883L_MODE       0x02
#define HMC5883L_DATA_X_H   0x03

class HMC5883L {
public:
    HMC5883L() : magX(0), magY(0), magZ(0), 
                  offsetX(0), offsetY(0), offsetZ(0),
                  scaleX(1.0f), scaleY(1.0f), scaleZ(1.0f) {}
    
    // Initialize the magnetometer
    bool begin() {
        Wire.beginTransmission(HMC5883L_ADDR);
        Wire.write(HMC5883L_CONFIG_A);
        Wire.write(0x70);  // 8 samples avg, 15Hz output rate, normal measurement
        if (Wire.endTransmission() != 0) return false;
        
        Wire.beginTransmission(HMC5883L_ADDR);
        Wire.write(HMC5883L_CONFIG_B);
        Wire.write(0x20);  // Gain = 1.3 Gauss (±1.3 Ga range)
        Wire.endTransmission();
        
        Wire.beginTransmission(HMC5883L_ADDR);
        Wire.write(HMC5883L_MODE);
        Wire.write(0x00);  // Continuous measurement mode
        Wire.endTransmission();
        
        delay(100);
        return true;
    }
    
    // Read magnetometer data
    void update() {
        Wire.beginTransmission(HMC5883L_ADDR);
        Wire.write(HMC5883L_DATA_X_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)HMC5883L_ADDR, (uint8_t)6, (uint8_t)true);
        
        if (Wire.available() == 6) {
            // Note: HMC5883L data order is X, Z, Y (not X, Y, Z!)
            magX = (int16_t)(Wire.read() << 8 | Wire.read());
            magZ = (int16_t)(Wire.read() << 8 | Wire.read());
            magY = (int16_t)(Wire.read() << 8 | Wire.read());
        }
    }
    
    // Get calibrated values (in microteslas after calibration)
    float getMagX() const { return (magX - offsetX) * scaleX; }
    float getMagY() const { return (magY - offsetY) * scaleY; }
    float getMagZ() const { return (magZ - offsetZ) * scaleZ; }
    
    // Get raw values
    int16_t getRawMagX() const { return magX; }
    int16_t getRawMagY() const { return magY; }
    int16_t getRawMagZ() const { return magZ; }
    
    // Calculate heading in degrees (0-360)
    // Note: Only accurate when sensor is level!
    float getHeading() const {
        float heading = atan2f(getMagY(), getMagX()) * 180.0f / M_PI;
        if (heading < 0) heading += 360.0f;
        return heading;
    }
    
    // Calibration: Set hard-iron offsets
    void setOffsets(int16_t x, int16_t y, int16_t z) {
        offsetX = x;
        offsetY = y;
        offsetZ = z;
    }
    
    // Calibration: Set soft-iron scale factors
    void setScale(float x, float y, float z) {
        scaleX = x;
        scaleY = y;
        scaleZ = z;
    }
    
    // Simple calibration routine (rotate drone in all directions during this)
    void calibrate(int durationMs = 10000) {
        Serial.println("Magnetometer calibration starting...");
        Serial.println("Rotate the drone slowly in all directions!");
        
        int16_t minX = 32767, maxX = -32768;
        int16_t minY = 32767, maxY = -32768;
        int16_t minZ = 32767, maxZ = -32768;
        
        unsigned long startTime = millis();
        while (millis() - startTime < (unsigned long)durationMs) {
            update();
            
            if (magX < minX) minX = magX;
            if (magX > maxX) maxX = magX;
            if (magY < minY) minY = magY;
            if (magY > maxY) maxY = magY;
            if (magZ < minZ) minZ = magZ;
            if (magZ > maxZ) maxZ = magZ;
            
            if ((millis() - startTime) % 1000 == 0) {
                Serial.printf("Calibrating... %d/%d ms\n", 
                              (int)(millis() - startTime), durationMs);
            }
            delay(50);
        }
        
        // Calculate hard-iron offsets (center of the ellipsoid)
        offsetX = (maxX + minX) / 2;
        offsetY = (maxY + minY) / 2;
        offsetZ = (maxZ + minZ) / 2;
        
        // Calculate soft-iron scale factors (normalize to average radius)
        float avgDelta = ((maxX - minX) + (maxY - minY) + (maxZ - minZ)) / 3.0f;
        scaleX = avgDelta / (maxX - minX);
        scaleY = avgDelta / (maxY - minY);
        scaleZ = avgDelta / (maxZ - minZ);
        
        Serial.println("Magnetometer calibration complete!");
        Serial.printf("Offsets: X=%d, Y=%d, Z=%d\n", offsetX, offsetY, offsetZ);
        Serial.printf("Scale: X=%.3f, Y=%.3f, Z=%.3f\n", scaleX, scaleY, scaleZ);
    }
    
    // Calibration data
    int16_t offsetX, offsetY, offsetZ;
    float scaleX, scaleY, scaleZ;

private:
    int16_t magX, magY, magZ;
};

#endif // HMC5883L_H
