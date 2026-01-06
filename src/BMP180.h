#ifndef BMP180_H
#define BMP180_H

#include <Arduino.h>
#include <Wire.h>

// BMP180 I2C address
#define BMP180_ADDR         0x77

// BMP180 registers
#define BMP180_CHIP_ID      0xD0
#define BMP180_CTRL         0xF4
#define BMP180_DATA         0xF6
#define BMP180_CAL_AC1      0xAA

// Commands
#define BMP180_CMD_TEMP     0x2E
#define BMP180_CMD_PRESS_0  0x34  // Ultra low power
#define BMP180_CMD_PRESS_1  0x74  // Standard
#define BMP180_CMD_PRESS_2  0xB4  // High resolution
#define BMP180_CMD_PRESS_3  0xF4  // Ultra high resolution

class BMP180 {
public:
    BMP180() : oversampling(1), temperature(0), pressure(0), altitude(0),
               seaLevelPressure(101325.0f) {}
    
    // Initialize the barometer
    bool begin() {
        // Check chip ID
        Wire.beginTransmission(BMP180_ADDR);
        Wire.write(BMP180_CHIP_ID);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)1);
        
        if (Wire.available()) {
            uint8_t id = Wire.read();
            if (id != 0x55) {
                Serial.printf("BMP180: Wrong chip ID: 0x%02X (expected 0x55)\n", id);
                return false;
            }
        } else {
            return false;
        }
        
        // Read calibration data
        readCalibration();
        return true;
    }
    
    // Set oversampling mode (0-3, higher = more accurate but slower)
    void setOversampling(uint8_t oss) {
        oversampling = min((uint8_t)3, oss);
    }
    
    // Read temperature and pressure (blocking, takes ~30ms at oss=1)
    void update() {
        readTemperature();
        readPressure();
        calculateAltitude();
    }
    
    // Get temperature in Celsius
    float getTemperature() const { return temperature; }
    
    // Get pressure in Pascals
    float getPressure() const { return pressure; }
    
    // Get altitude in meters (relative to sea level pressure)
    float getAltitude() const { return altitude; }
    
    // Set sea level pressure for altitude calculation (in Pascals)
    // Call this with current local pressure at known altitude for relative altitude
    void setSeaLevelPressure(float pressure) {
        seaLevelPressure = pressure;
    }
    
    // Calibrate altitude to zero at current position
    void calibrateAltitude() {
        update();
        seaLevelPressure = pressure;
        altitude = 0;
        Serial.printf("BMP180: Sea level pressure set to %.2f Pa\n", seaLevelPressure);
    }

private:
    uint8_t oversampling;
    float temperature;
    float pressure;
    float altitude;
    float seaLevelPressure;
    
    // Calibration coefficients
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2, MB, MC, MD;
    int32_t B5;  // Shared between temp and pressure calc
    
    void readCalibration() {
        Wire.beginTransmission(BMP180_ADDR);
        Wire.write(BMP180_CAL_AC1);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)22);
        
        AC1 = (int16_t)(Wire.read() << 8 | Wire.read());
        AC2 = (int16_t)(Wire.read() << 8 | Wire.read());
        AC3 = (int16_t)(Wire.read() << 8 | Wire.read());
        AC4 = (uint16_t)(Wire.read() << 8 | Wire.read());
        AC5 = (uint16_t)(Wire.read() << 8 | Wire.read());
        AC6 = (uint16_t)(Wire.read() << 8 | Wire.read());
        B1 = (int16_t)(Wire.read() << 8 | Wire.read());
        B2 = (int16_t)(Wire.read() << 8 | Wire.read());
        MB = (int16_t)(Wire.read() << 8 | Wire.read());
        MC = (int16_t)(Wire.read() << 8 | Wire.read());
        MD = (int16_t)(Wire.read() << 8 | Wire.read());
    }
    
    void readTemperature() {
        // Start temperature measurement
        Wire.beginTransmission(BMP180_ADDR);
        Wire.write(BMP180_CTRL);
        Wire.write(BMP180_CMD_TEMP);
        Wire.endTransmission();
        delay(5);  // Wait for conversion
        
        // Read raw temperature
        Wire.beginTransmission(BMP180_ADDR);
        Wire.write(BMP180_DATA);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)2);
        
        int32_t UT = (int32_t)(Wire.read() << 8 | Wire.read());
        
        // Calculate true temperature
        int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
        int32_t X2 = ((int32_t)MC << 11) / (X1 + (int32_t)MD);
        B5 = X1 + X2;
        temperature = ((B5 + 8) >> 4) / 10.0f;
    }
    
    void readPressure() {
        // Start pressure measurement
        uint8_t cmd = BMP180_CMD_PRESS_0 + (oversampling << 6);
        Wire.beginTransmission(BMP180_ADDR);
        Wire.write(BMP180_CTRL);
        Wire.write(cmd);
        Wire.endTransmission();
        
        // Wait for conversion (depends on oversampling)
        delay(2 + (3 << oversampling));
        
        // Read raw pressure (up to 3 bytes)
        Wire.beginTransmission(BMP180_ADDR);
        Wire.write(BMP180_DATA);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)3);
        
        int32_t UP = (((int32_t)Wire.read() << 16) | 
                      ((int32_t)Wire.read() << 8) | 
                      (int32_t)Wire.read()) >> (8 - oversampling);
        
        // Calculate true pressure (algorithm from datasheet)
        int32_t B6 = B5 - 4000;
        int32_t X1 = ((int32_t)B2 * ((B6 * B6) >> 12)) >> 11;
        int32_t X2 = ((int32_t)AC2 * B6) >> 11;
        int32_t X3 = X1 + X2;
        int32_t B3 = ((((int32_t)AC1 * 4 + X3) << oversampling) + 2) >> 2;
        
        X1 = ((int32_t)AC3 * B6) >> 13;
        X2 = ((int32_t)B1 * ((B6 * B6) >> 12)) >> 16;
        X3 = ((X1 + X2) + 2) >> 2;
        uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
        uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> oversampling);
        
        int32_t p;
        if (B7 < 0x80000000) {
            p = (B7 << 1) / B4;
        } else {
            p = (B7 / B4) << 1;
        }
        
        X1 = (p >> 8) * (p >> 8);
        X1 = (X1 * 3038) >> 16;
        X2 = (-7357 * p) >> 16;
        pressure = p + ((X1 + X2 + 3791) >> 4);
    }
    
    void calculateAltitude() {
        // Barometric formula
        altitude = 44330.0f * (1.0f - powf(pressure / seaLevelPressure, 0.1903f));
    }
};

#endif // BMP180_H
