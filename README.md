# ESP32 Drone Flight Controller

A complete flight controller for ESP32-based quadcopter drones with sensor fusion for stable orientation control.

![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Framework](https://img.shields.io/badge/Framework-Arduino-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Features

- 🎯 **Mahony AHRS Filter** - Quaternion-based sensor fusion (no gimbal lock)
- 📡 **Multi-Sensor Support**
  - MPU6050 IMU (accelerometer + gyroscope)
  - HMC5883L Magnetometer (compass heading)
  - BMP180 Barometer (altitude)
- 🎮 **PID Control** - Independent roll, pitch, yaw controllers with anti-windup
- 🌐 **Web Interface** - Real-time PID tuning via WebSocket
- ⚡ **500Hz Control Loop** - Hardware timer driven for consistent timing
- 🔧 **Configurable** - All parameters in single config file

## Hardware Requirements

| Component | Model | I2C Address |
|-----------|-------|-------------|
| IMU | MPU6050 | 0x68 |
| Magnetometer | HMC5883L | 0x1E |
| Barometer | BMP180 | 0x77 |
| Microcontroller | ESP32 DevKit | - |
| ESCs | Any PWM ESC | - |

### Wiring

| Function | ESP32 Pin |
|----------|-----------|
| I2C SDA | GPIO 21 |
| I2C SCL | GPIO 22 |
| Motor Front-Left | GPIO 32 |
| Motor Front-Right | GPIO 33 |
| Motor Back-Left | GPIO 25 |
| Motor Back-Right | GPIO 26 |

## Installation

1. Install [PlatformIO](https://platformio.org/) in VS Code
2. Clone this repository
3. Open the project folder in VS Code
4. Click ✓ to build, → to upload

## Usage

### Serial Commands (115200 baud)

```
arm            - Arm motors (throttle must be low)
disarm         - Disarm motors
status         - Show orientation and sensor data
throttle N     - Set throttle (1000-1800)
calibrate_mag  - Calibrate magnetometer
calibrate_baro - Zero altitude reference
```

### Web Interface

1. Connect to WiFi AP: **DroneTuning** (password: `12345678`)
2. Open browser to `http://192.168.4.1`
3. Adjust PID gains in real-time

## Project Structure

```
src/
├── main.cpp          # Flight control loop
├── config.h          # Pin definitions, PID gains
├── IMU_Driver.*      # MPU6050 driver
├── HMC5883L.h        # Magnetometer driver
├── BMP180.h          # Barometer driver
├── MahonyFilter.*    # Sensor fusion
├── PIDController.*   # PID with anti-windup
├── MotorControl.*    # Quadcopter X mixing
├── LowPassFilter.h   # Noise filtering
└── WebServer.*       # PID tuning UI
```

## PID Tuning Guide

1. Start with **P only** (Ki=0, Kd=0)
2. Increase P until oscillation, reduce by 30%
3. Add small **D** to dampen oscillations
4. Add small **I** only if drift occurs
5. Keep **I-Max** low to prevent windup

## Safety

⚠️ **Always test with propellers removed first!**

- Use a thrust test stand for initial testing
- Keep throttle low when arming
- Have a kill switch ready

## License

MIT License - feel free to use and modify!

## Contributing

Pull requests welcome! Please open an issue first to discuss changes.
