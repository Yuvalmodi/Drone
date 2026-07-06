# ESP32 Quadcopter with VL53L0X LiDAR & FlySky FS-i4 Control

An open-source ESP32-based quadcopter flight controller featuring **dual-core FreeRTOS task scheduling**, **PID flight stabilization**, **FlySky FS-i4 manual radio control**, and **VL53L0X LiDAR-based obstacle detection**. The project separates flight-critical control from sensor monitoring to achieve stable real-time performance while allowing future autonomous safety features.

---

# 🚀 System Architecture & Dual-Core Workflow

The ESP32 uses both processor cores through **FreeRTOS Tasks**.

```text
                           ┌──────────────────────────┐
                           │   ESP32 Dual-Core CPU    │
                           └────────────┬─────────────┘
                                        │
                ┌───────────────────────┴───────────────────────┐
                ▼                                               ▼
   ┌──────────────────────────┐                    ┌──────────────────────────┐
   │          CORE 0          │                    │          CORE 1          │
   │  Safety & Radio Tasks    │                    │  Flight Control Tasks    │
   └────────────┬─────────────┘                    └────────────┬─────────────┘
                │                                               │
                ├► VL53L0X Distance Reading                     ├► MPU6050 Reading
                ├► FlySky PWM Parsing                           ├► Sensor Fusion
                ├► Obstacle Detection                           ├► PID Controller
                └► Safety Override Flag                         └► Motor PWM Output
```

---

# Flight-Critical Stabilization Loop (Core 1)

Runs at approximately **250Hz (every 4ms)**.

### Responsibilities

- Read MPU6050 Gyroscope & Accelerometer
- Calculate Roll, Pitch and Yaw
- Sensor Fusion
- PID Computation
- Motor Mixing
- Generate PWM signals for ESCs

---

# Safety & Radio Monitoring (Core 0)

Runs independently from the stabilization loop.

### Responsibilities

- Read FlySky receiver channels
- Read VL53L0X LiDAR distance
- Detect nearby obstacles
- Set Safety Override Flag
- Send telemetry/debug information

If an obstacle is detected within the configured safety distance, Core 0 sets:

```cpp
activeSafetyOverride = true;
```

Core 1 then reduces throttle and applies reverse pitch correction to slow the drone.

---

# 🛠 Hardware Components

| Component | Specification | Purpose |
|------------|--------------|---------|
| ESP32 DevKit V4 | Dual-Core 240MHz | Main Flight Controller |
| MPU6050 | 6-Axis IMU | Roll, Pitch & Yaw Estimation |
| VL53L0X | Time-of-Flight LiDAR | Distance Measurement |
| FlySky FS-i4 | 2.4GHz RC System | Manual Flight Control |
| 2212 BLDC Motors | 1000KV / 1400KV | Drone Propulsion |
| 30A ESC | PWM Controlled | BLDC Speed Controller |
| F450 Frame | Glass Fiber | Mechanical Structure |

---

# 🔌 ESP32 Pin Mapping

| Device | ESP32 GPIO | Signal |
|---------|------------|---------|
| MPU6050 SDA | GPIO21 | I2C Data |
| MPU6050 SCL | GPIO22 | I2C Clock |
| VL53L0X SDA | GPIO21 | Shared I2C |
| VL53L0X SCL | GPIO22 | Shared I2C |
| FlySky CH1 (Roll) | GPIO16 | PWM Input |
| FlySky CH2 (Pitch) | GPIO17 | PWM Input |
| FlySky CH3 (Throttle) | GPIO18 | PWM Input |
| FlySky CH4 (Yaw) | GPIO19 | PWM Input |
| ESC Front Left | GPIO13 | PWM Output |
| ESC Front Right | GPIO12 | PWM Output |
| ESC Rear Left | GPIO14 | PWM Output |
| ESC Rear Right | GPIO27 | PWM Output |

---

# Hardware Connection Diagram

```text
                    ESP32 Dev Module

                  +----------------------+

      MPU6050 SDA ---- GPIO21
      MPU6050 SCL ---- GPIO22

      VL53L0X SDA ---- GPIO21
      VL53L0X SCL ---- GPIO22

      FlySky CH1 ----- GPIO16
      FlySky CH2 ----- GPIO17
      FlySky CH3 ----- GPIO18
      FlySky CH4 ----- GPIO19

      ESC1 ---------- GPIO13
      ESC2 ---------- GPIO12
      ESC3 ---------- GPIO14
      ESC4 ---------- GPIO27

                  +----------------------+
```

---

# PID Flight Stabilization

The PID controller continuously compares the desired attitude with the measured attitude.

```text
Error = Desired Angle − Current Angle

Integral += Error × dt

Derivative = (Error − Previous Error) / dt

Correction =
Kp × Error +
Ki × Integral +
Kd × Derivative
```

Motor speed is updated using

```text
Motor Speed = Base Throttle + PID Correction
```

---

# Sensor Fusion

The MPU6050 contains

- Accelerometer
- Gyroscope

Accelerometer provides stable orientation.

Gyroscope provides fast angular velocity.

A complementary/Kalman filter combines both to obtain a stable estimate.

---

# LiDAR Safety Logic

```cpp
distance = readVL53L0X();

if(distance < SAFE_DISTANCE)
{
    activeSafetyOverride = true;
}
else
{
    activeSafetyOverride = false;
}
```

When active, the controller reduces throttle and applies reverse pitch correction.

---

# Motor Mixing

Each motor receives a different PWM value.

```text
Front Left
Throttle + Pitch - Roll + Yaw

Front Right
Throttle + Pitch + Roll - Yaw

Rear Left
Throttle - Pitch - Roll - Yaw

Rear Right
Throttle - Pitch + Roll + Yaw
```

---

# Communication Protocols

| Device | Protocol |
|---------|----------|
| MPU6050 | I2C |
| VL53L0X | I2C |
| FlySky Receiver | PWM |
| ESC | PWM |
| ESP32 Programming | USB UART |

---

# Bench Testing Safety

## ⚠ Remove Propellers

Always remove propellers before

- Uploading firmware
- ESC calibration
- PID tuning
- Sensor testing

2212 BLDC motors can exceed **10,000 RPM**.

---

# ESC Calibration

Calibrate all ESCs before first flight.

Pulse Width

Minimum

```
1000 µs
```

Maximum

```
2000 µs
```

---

# FlySky Safety

- Turn transmitter ON before powering the drone.
- Configure receiver failsafe.
- Failsafe throttle should be **1000µs**.

---

# Required Libraries

Install using Arduino Library Manager.

- Adafruit MPU6050
- Adafruit VL53L0X
- Adafruit Unified Sensor
- Wire
- ESP32 Arduino Core

---

# Installation

1. Clone this repository.

```bash
git clone https://github.com/yourusername/ESP32-Quadcopter.git
```

2. Open the project in Arduino IDE.

3. Select

```
Board
↓

ESP32 Dev Module
```

4. Configure

```
CPU Frequency : 240 MHz

Flash Frequency : 80 MHz

Core Debug Level : None
```

5. Connect ESP32.

6. Select COM Port.

7. Upload firmware.

8. Open Serial Monitor

```
115200 Baud
```

---

# Features

- Dual-Core FreeRTOS
- PID Flight Stabilization
- FlySky FS-i4 Manual Control
- MPU6050 Sensor Fusion
- VL53L0X Distance Monitoring
- 400Hz PWM ESC Control
- Future Collision Avoidance Support
- Modular Open-Source Design

---

# Future Improvements

- GPS Navigation
- Autonomous Waypoint Flight
- Optical Flow Sensor
- Barometer Altitude Hold
- Loiter Mode
- Return To Home (RTH)
- FPV Camera Integration
- Telemetry over Wi-Fi
- Mobile App Control

---
