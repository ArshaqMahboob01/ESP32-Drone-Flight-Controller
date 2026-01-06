#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// PROJECT INFO
// ============================================================================
#define PROJECT_NAME "ESP32_Drone_Controller"
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

// ============================================================================
// DEBUG SETTINGS
// ============================================================================
#define ENABLE_DEBUG 1
#define SERIAL_BAUD 115200

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// I2C for IMU (MPU6050)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLOCK   400000  // 400kHz fast mode

// Motor PWM pins (adjust for your wiring)
#define MOTOR_PIN_FL 32  // Front-Left
#define MOTOR_PIN_FR 33  // Front-Right
#define MOTOR_PIN_BL 25  // Back-Left
#define MOTOR_PIN_BR 26  // Back-Right

// ============================================================================
// LOOP TIMING
// ============================================================================
#define LOOP_FREQUENCY_HZ   500     // Main loop frequency
#define LOOP_PERIOD_US      (1000000 / LOOP_FREQUENCY_HZ)  // 2000us for 500Hz

// ============================================================================
// IMU CONFIGURATION (MPU6050)
// ============================================================================
#define MPU6050_ADDRESS     0x68
#define GYRO_SCALE          131.0f   // LSB/(°/s) for ±250°/s range
#define ACCEL_SCALE         16384.0f // LSB/g for ±2g range

// Calibration samples (drone must be stationary)
#define CALIBRATION_SAMPLES 2000

// ============================================================================
// MAGNETOMETER CONFIGURATION (HMC5883L)
// ============================================================================
#define HMC5883L_ADDRESS    0x1E
#define MAG_DECLINATION     0.0f     // Magnetic declination for your location (degrees)
                                     // Find yours at: https://www.ngdc.noaa.gov/geomag/declination.shtml

// ============================================================================
// BAROMETER CONFIGURATION (BMP180)
// ============================================================================
#define BMP180_ADDRESS      0x77
#define BMP180_OVERSAMPLING 1        // 0=ultra low power, 1=standard, 2=high, 3=ultra high
#define BARO_UPDATE_HZ      25       // Barometer update rate (Hz) - slower than main loop

// ============================================================================
// FILTER SETTINGS
// ============================================================================
// Low-pass filter cutoff frequencies (Hz)
#define GYRO_LPF_CUTOFF     100.0f   // Gyro filter cutoff
#define ACCEL_LPF_CUTOFF    50.0f    // Accel filter cutoff (lower = more smooth)

// ============================================================================
// MAHONY FILTER TUNING
// ============================================================================
#define MAHONY_KP   1.0f    // Proportional gain (higher = faster convergence, more noise)
#define MAHONY_KI   0.0f    // Integral gain (helps with gyro drift, start at 0)

// ============================================================================
// PID TUNING (CONSERVATIVE DEFAULTS - TUNE CAREFULLY!)
// ============================================================================
// Roll PID
#define ROLL_KP     1.0f
#define ROLL_KI     0.0f
#define ROLL_KD     0.0f
#define ROLL_IMAX   50.0f   // Integral windup limit

// Pitch PID
#define PITCH_KP    1.0f
#define PITCH_KI    0.0f
#define PITCH_KD    0.0f
#define PITCH_IMAX  50.0f

// Yaw PID
#define YAW_KP      2.0f
#define YAW_KI      0.0f
#define YAW_KD      0.0f
#define YAW_IMAX    50.0f

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
#define PWM_FREQUENCY   250     // ESC PWM frequency (Hz)
#define PWM_RESOLUTION  12      // 12-bit resolution (0-4095)
#define PWM_MIN         1000    // Min pulse width (μs) - motor off
#define PWM_MAX         2000    // Max pulse width (μs) - full throttle

// Motor mixing for quadcopter X configuration
// Throttle + Roll + Pitch + Yaw
// FL: +T -R +P -Y
// FR: +T +R +P +Y
// BL: +T -R -P +Y
// BR: +T +R -P -Y

// ============================================================================
// SAFETY SETTINGS
// ============================================================================
#define THROTTLE_IDLE       1050    // Idle throttle (motors spinning slowly)
#define THROTTLE_MIN        1000    // Minimum throttle (disarmed)
#define THROTTLE_MAX        1800    // Maximum throttle (leave headroom)
#define ARM_THROTTLE_MAX    1050    // Max throttle to allow arming

#endif // CONFIG_H
