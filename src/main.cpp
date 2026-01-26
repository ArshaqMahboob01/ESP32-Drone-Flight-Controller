/*
 * ESP32 Drone Flight Controller
 * 
 * Main flight control loop with:
 * - MPU6050 IMU for orientation sensing
 * - HMC5883L magnetometer for yaw correction
 * - BMP180 barometer for altitude hold
 * - Mahony filter for sensor fusion
 * - PID control for roll, pitch, yaw
 * - Motor mixing for quadcopter X configuration
 * - Web server for real-time PID tuning
 */

#include <Arduino.h>
#include "config.h"
#include "IMU_Driver.h"
#include "HMC5883L.h"
#include "BMP180.h"
#include "MahonyFilter.h"
#include "LowPassFilter.h"
#include "PIDController.h"
#include "MotorControl.h"
#include "SettingsManager.h"
#include "WebServer.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Sensors
IMUDriver imu;
HMC5883L mag;
BMP180 baro;

// Filters
MahonyFilter ahrs(MAHONY_KP, MAHONY_KI);
LowPassFilter gyroFilterX, gyroFilterY, gyroFilterZ;
LowPassFilter accelFilterX, accelFilterY, accelFilterZ;
LowPassFilter altitudeFilter;

// Controllers
PIDController rollPID(ROLL_KP, ROLL_KI, ROLL_KD, ROLL_IMAX);
PIDController pitchPID(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_IMAX);
PIDController yawPID(YAW_KP, YAW_KI, YAW_KD, YAW_IMAX);

// Motor output
MotorControl motors;

// Web server for tuning
TuningWebServer webServer(80);

// Persistence
SettingsManager droneSettings;

// ============================================================================
// FLIGHT CONTROL VARIABLES
// ============================================================================

volatile bool flightLoopFlag = false;
hw_timer_t* flightTimer = nullptr;

// Timing
unsigned long lastLoopTime = 0;
unsigned long loopDurationUs = 0;
unsigned long lastBaroUpdate = 0;
unsigned long lastMagUpdate = 0;

// State
bool armed = false;
float throttleSetpoint = THROTTLE_MIN;
float rollSetpoint = 0.0f;
float pitchSetpoint = 0.0f;
float yawSetpoint = 0.0f;

// Current orientation
float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

// Barometer data
float currentAltitude = 0.0f;
float currentTemperature = 0.0f;
float currentPressure = 0.0f;

// Magnetometer data
float magHeading = 0.0f;

// Sensor availability flags
bool magAvailable = false;
bool baroAvailable = false;

// ============================================================================
// TIMER INTERRUPT
// ============================================================================

void IRAM_ATTR onFlightTimer() {
    flightLoopFlag = true;
}

// ============================================================================
// WIFI CONFIGURATION - CHANGE THESE!
// ============================================================================

const char* WIFI_SSID = "4G-MIFI-3F9A";     // Change to your WiFi SSID
const char* WIFI_PASS = "1234567890"; // Change to your WiFi password

// Set to true to create an Access Point instead of connecting to WiFi
const bool USE_AP_MODE = true;
const char* AP_SSID = "DroneTuning";
const char* AP_PASS = "12345678";  // Min 8 characters, or nullptr for open

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("    ESP32 Drone Flight Controller");
    Serial.printf("    Version %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    Serial.println("========================================\n");
    
    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK);
    
    // Initialize IMU (MPU6050)
    Serial.println("[INIT] Starting IMU (MPU6050)...");
    if (!imu.begin()) {
        Serial.println("[ERROR] IMU initialization failed!");
        while (1) { delay(1000); }
    }
    Serial.println("[OK] IMU initialized");
    
    // Initialize Magnetometer (HMC5883L)
    Serial.println("[INIT] Starting Magnetometer (HMC5883L)...");
    if (mag.begin()) {
        magAvailable = true;
        Serial.println("[OK] Magnetometer initialized");
    } else {
        Serial.println("[WARN] Magnetometer not found - yaw will drift over time");
    }
    
    // Initialize Barometer (BMP180)
    Serial.println("[INIT] Starting Barometer (BMP180)...");
    if (baro.begin()) {
        baroAvailable = true;
        baro.setOversampling(BMP180_OVERSAMPLING);
        Serial.println("[OK] Barometer initialized");
    } else {
        Serial.println("[WARN] Barometer not found - altitude hold unavailable");
    }
    
    // Calibrate IMU
    Serial.println("[INIT] Calibrating IMU (keep drone level and still)...");
    imu.calibrate(CALIBRATION_SAMPLES);
    
    // Calibrate barometer altitude
    if (baroAvailable) {
        Serial.println("[INIT] Calibrating barometer...");
        baro.calibrateAltitude();
    }
    
    // Initialize filters
    Serial.println("[INIT] Setting up filters...");
    gyroFilterX.begin(GYRO_LPF_CUTOFF, LOOP_FREQUENCY_HZ);
    gyroFilterY.begin(GYRO_LPF_CUTOFF, LOOP_FREQUENCY_HZ);
    gyroFilterZ.begin(GYRO_LPF_CUTOFF, LOOP_FREQUENCY_HZ);
    accelFilterX.begin(ACCEL_LPF_CUTOFF, LOOP_FREQUENCY_HZ);
    accelFilterY.begin(ACCEL_LPF_CUTOFF, LOOP_FREQUENCY_HZ);
    accelFilterZ.begin(ACCEL_LPF_CUTOFF, LOOP_FREQUENCY_HZ);
    altitudeFilter.begin(5.0f, BARO_UPDATE_HZ);  // 5Hz LPF on altitude
    
    // Initialize motors
    Serial.println("[INIT] Initializing motor outputs...");
    motors.begin();
    
    // Initialize settings and load saved values
    Serial.println("[INIT] Loading settings from flash...");
    droneSettings.begin();
    droneSettings.loadPID("roll", rollPID);
    droneSettings.loadPID("pitch", pitchPID);
    droneSettings.loadPID("yaw", yawPID);
    droneSettings.loadIMUCalibration(imu);
    droneSettings.loadMagCalibration(mag);
    
    // Initialize web server
    Serial.println("[INIT] Starting web server...");
    if (USE_AP_MODE) {
        webServer.beginAP(AP_SSID, AP_PASS);
    } else {
        webServer.begin(WIFI_SSID, WIFI_PASS);
    }
    webServer.setPIDControllers(&rollPID, &pitchPID, &yawPID);
    webServer.setSettingsManager(&droneSettings); // Pass settings manager to web server
    
    // Configure PID output limits
    rollPID.setOutputLimits(-500, 500);
    pitchPID.setOutputLimits(-500, 500);
    yawPID.setOutputLimits(-500, 500);
    
    // Setup hardware timer for flight loop
    Serial.println("[INIT] Configuring flight loop timer...");
    // ESP32 Arduino v3.x API
    flightTimer = timerBegin(1000000);  // 1MHz timer (1us tick)
    timerAttachInterrupt(flightTimer, &onFlightTimer);
    timerAlarm(flightTimer, LOOP_PERIOD_US, true, 0);  // period, autoreload, unlimited
    
    Serial.println("\n[READY] Flight controller initialized!");
    Serial.printf("[INFO] Loop frequency: %d Hz\n", LOOP_FREQUENCY_HZ);
    Serial.printf("[INFO] Sensors: IMU=%s, MAG=%s, BARO=%s\n", 
                  "OK", magAvailable ? "OK" : "N/A", baroAvailable ? "OK" : "N/A");
    Serial.println("[INFO] Web interface available via:");
    Serial.printf("  - http://%s\n", webServer.getIP().c_str());
    Serial.println("  - http://drone.local");
    if (USE_AP_MODE) {
        Serial.printf("[IMPORTANT] Connect your device to WiFi: %s\n", AP_SSID);
    }
    Serial.println("\n[SAFETY] Motors are DISARMED");
    Serial.println("[INFO] Commands (or use numbers 1-6):");
    Serial.println("  1: arm           2: disarm        3: status");
    Serial.println("  4: calibrate_mag 5: calibrate_baro 6 <val>: throttle");
    Serial.println("  7: calibrate_imu (ensure drone is still and level)\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Handle flight control at fixed rate
    if (flightLoopFlag) {
        flightLoopFlag = false;
        
        unsigned long loopStart = micros();
        
        // Calculate delta time
        unsigned long now = micros();
        float dt = (now - lastLoopTime) / 1000000.0f;
        lastLoopTime = now;
        
        // Sanity check dt
        if (dt <= 0 || dt > 0.1f) {
            dt = 1.0f / LOOP_FREQUENCY_HZ;
        }
        
        // ----------------------------------------------------------------
        // 1. READ IMU (every loop - 500Hz)
        // ----------------------------------------------------------------
        imu.update();
        
        // Get raw values and apply low-pass filter
        float gx = gyroFilterX.update(imu.getGyroX());
        float gy = gyroFilterY.update(imu.getGyroY());
        float gz = gyroFilterZ.update(imu.getGyroZ());
        
        float ax = accelFilterX.update(imu.getAccelX());
        float ay = accelFilterY.update(imu.getAccelY());
        float az = accelFilterZ.update(imu.getAccelZ());
        
        // ----------------------------------------------------------------
        // 2. READ MAGNETOMETER (at lower rate - ~75Hz)
        // ----------------------------------------------------------------
        if (magAvailable && (now - lastMagUpdate > 13333)) {  // ~75Hz
            lastMagUpdate = now;
            mag.update();
            magHeading = mag.getHeading() + MAG_DECLINATION;
            if (magHeading > 360) magHeading -= 360;
            if (magHeading < 0) magHeading += 360;
        }
        
        // ----------------------------------------------------------------
        // 3. READ BAROMETER (at lower rate - configurable)
        // ----------------------------------------------------------------
        if (baroAvailable && (now - lastBaroUpdate > (1000000 / BARO_UPDATE_HZ))) {
            lastBaroUpdate = now;
            baro.update();
            currentAltitude = altitudeFilter.update(baro.getAltitude());
            currentTemperature = baro.getTemperature();
            currentPressure = baro.getPressure();
        }
        
        // ----------------------------------------------------------------
        // 4. UPDATE SENSOR FUSION (convert gyro to rad/s)
        // ----------------------------------------------------------------
        ahrs.update(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                    ax, ay, az, dt);
        
        currentRoll = ahrs.getRoll();
        currentPitch = ahrs.getPitch();
        currentYaw = ahrs.getYaw();
        
        // If magnetometer available, use it to correct yaw drift
        // (Simple approach: blend with Mahony yaw)
        if (magAvailable) {
            // TODO: Implement proper tilt-compensated heading
            // For now, magnetometer heading is available for reference
        }
        
        // ----------------------------------------------------------------
        // 5. RUN PID CONTROLLERS
        // ----------------------------------------------------------------
        float rollError = rollSetpoint - currentRoll;
        float pitchError = pitchSetpoint - currentPitch;
        float yawError = yawSetpoint - currentYaw;
        
        // Normalize yaw error to -180 to 180
        while (yawError > 180) yawError -= 360;
        while (yawError < -180) yawError += 360;
        
        // Use gyro rates for D-term (avoids derivative kick)
        float rollOutput = rollPID.compute(rollError, gx, dt);
        float pitchOutput = pitchPID.compute(pitchError, gy, dt);
        float yawOutput = yawPID.compute(yawError, gz, dt);
        
        // ----------------------------------------------------------------
        // 6. UPDATE MOTORS
        // ----------------------------------------------------------------
        if (armed) {
            motors.update(throttleSetpoint, rollOutput, pitchOutput, yawOutput);
        } else {
            motors.stop();
        }
        
        // Calculate loop duration
        loopDurationUs = micros() - loopStart;
        
        // ----------------------------------------------------------------
        // 7. UPDATE TELEMETRY (rate limited internally)
        // ----------------------------------------------------------------
        webServer.updateTelemetry(currentRoll, currentPitch, currentYaw,
                                   gx, gy, gz, loopDurationUs, armed);
        webServer.updateExtendedTelemetry(currentAltitude, magHeading, currentTemperature);
    }
    
    // Handle serial commands (non-blocking)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        
        if (cmd == "arm" || cmd == "1") {
            if (throttleSetpoint <= ARM_THROTTLE_MAX) {
                armed = true;
                motors.arm();
                rollPID.reset();
                pitchPID.reset();
                yawPID.reset();
                yawSetpoint = currentYaw;  // Lock current yaw
                Serial.println("[ARM] Motors ARMED!");
            } else {
                Serial.println("[ERROR] Lower throttle to arm!");
            }
        }
        else if (cmd == "disarm" || cmd == "2") {
            armed = false;
            motors.disarm();
            Serial.println("[DISARM] Motors DISARMED");
        }
        else if (cmd == "status" || cmd == "3") {
            Serial.println("\n--- STATUS ---");
            Serial.printf("Armed: %s\n", armed ? "YES" : "NO");
            Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", 
                          currentRoll, currentPitch, currentYaw);
            if (magAvailable) {
                Serial.printf("Mag Heading: %.1f°\n", magHeading);
            }
            if (baroAvailable) {
                Serial.printf("Altitude: %.2f m, Temp: %.1f°C, Press: %.0f Pa\n",
                               currentAltitude, currentTemperature, currentPressure);
            }
            Serial.printf("Loop time: %lu us\n", loopDurationUs);
            Serial.printf("Throttle: %.0f\n", throttleSetpoint);
            Serial.println("--------------\n");
        }
        else if (cmd == "calibrate_mag" || cmd == "4") {
            if (magAvailable) {
                Serial.println("Starting magnetometer calibration...");
                mag.calibrate(15000);  // 15 seconds
                droneSettings.saveMagCalibration(mag);
                Serial.println("Magnetometer calibration saved!");
            } else {
                Serial.println("[ERROR] Magnetometer not available");
            }
        }
        else if (cmd == "calibrate_baro" || cmd == "5") {
            if (baroAvailable) {
                baro.calibrateAltitude();
            } else {
                Serial.println("[ERROR] Barometer not available");
            }
        }
        else if (cmd == "calibrate_imu" || cmd == "7") {
            Serial.println("Starting IMU calibration... DO NOT MOVE DRONE!");
            imu.calibrate(2000);
            droneSettings.saveIMUCalibration(imu);
            Serial.println("IMU calibration saved to flash!");
        }
        else if (cmd.startsWith("throttle ") || cmd.startsWith("6 ")) {
            int spaceIdx = cmd.indexOf(' ');
            if (spaceIdx != -1) {
                throttleSetpoint = cmd.substring(spaceIdx + 1).toFloat();
                throttleSetpoint = constrain(throttleSetpoint, THROTTLE_MIN, THROTTLE_MAX);
                Serial.printf("Throttle set to: %.0f\n", throttleSetpoint);
            }
        }
    }
    
    // Small delay to prevent watchdog issues and allow WiFi to work
    delay(1);
}
