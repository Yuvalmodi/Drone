# ESP32 Autonomous Quadcopter with LiDAR Obstacle Avoidance

An open-source, dual-core autonomous flight stabilization and collision avoidance system designed for hobby-scale quadcopters. This platform utilizes the ESP32's native FreeRTOS capabilities to separate high-frequency flight mechanics from environmental sensor polling, implementing a real-time Proportional-Integral-Derivative (PID) stabilization matrix and intelligent automated braking using a forward-facing LiDAR sensor.

---

## 🚀 System Architecture & Dual-Core Workflow

To ensure flight stability is never compromised by input/output latency, this system splits operational code execution across both physical processing cores of the ESP32 chip using **FreeRTOS Tasks**.

```
                           ┌──────────────────────────┐
                           │   ESP32 Dual-Core CPU    │
                           └────────────┬─────────────┘
                                        │
                ┌───────────────────────┴───────────────────────┐
                ▼                                               ▼
   ┌──────────────────────────┐                    ┌──────────────────────────┐
   │          CORE 0          │                    │          CORE 1          │
   │  Secondary Safety Tasks  │                    │   Flight-Critical Tasks  │
   └────────────┬─────────────┘                    └────────────┬─────────────┘
                │                                               │
                ├► LiDAR Data Ingestion (UART2)                 ├► MPU6050 6-Axis IMU Reading
                ├► Proximity Logic Calculation                  ├► Angle / Attenuation Drift
                └► Safety Brake Override Flag                   ├► Active PID Calculations
                                                                └► 400Hz Motor Mixing Out
```

### 1. Flight-Critical Stabilization Loop (Core 1)
* **High Frequency Priority:** Runs continuously at ~250Hz (every 4 milliseconds) to ensure near-instantaneous flight physical adjustments.
* **Sensor Fusion:** Polls the MPU6050 6-axis accelerometer/gyroscope over a dedicated high-speed I2C bus.
* **PID Computation:** Compares raw orientation vectors against flat reference benchmarks (0.0-degree pitch and roll angles) to derive instant correction signals.
* **Motor Mixing Array:** Combines baseline throttle variables with positional correction values, translating them into dedicated 12-bit LEDC PWM microsecond pulses mapped directly to four independent 2212 BLDC motors.

### 2. Autonomous Safety & Proximity Monitoring (Core 0)
* **Isolated Asynchronous I/O:** Runs as a secondary priority background task to ensure slow data packets from external components never pause or stall the flight stabilization engine.
* **LiDAR Parsing:** Continuously opens an asynchronous serial stream over UART2 to ingest raw, high-speed distance telemetry frames from a forward-facing mini LiDAR module.
* **Proximity Boundary Matrix:** Evaluates target clearance vectors against a configurable threshold (50 cm).
* **Active Override Intercept:** If an incoming obstacle violates safety tolerances, the task sets a global atomic volatile flag (`activeSafetyOverride = true`). Core 1 intercepts this signal, scales back nominal throttle calculations to trim velocity, and applies a reverse pitch adjustment to steer the quadcopter backwards, preventing a crash.

---

## 🛠️ Detailed Hardware Component Matrix

| Component Group | Component Name | Technical Specification | Operational Purpose |
| :--- | :--- | :--- | :--- |
| **Main Processing Unit**| ESP32 DevKitC v4 | WROOM-32D / 240MHz Dual Core | System brain running multitasking loops. |
| **Inertial Measurement**| MPU6050 IMU Breakout | 6-Axis Accel/Gyro, I2C Interface | Monitors pitch, roll, and attitude kinetics. |
| **Distance Ranging** | TF-Luna or TF-Mini S | Mini LiDAR, UART/Serial Interface | Monitors forward clearance vectors. |
| **Propulsion Motors** | Generic 2212 Brushless | 1000KV or 1400KV BLDC Variant | Provides flight lift and kinetic control vectors. |
| **Speed Regulation** | 30A Electronic Speed Controller (ESC) | BEC Support, 400Hz+ PWM Input | Decodes microcontroller commands into high-power BLDC phases. |
| **Structural Frame** | F450 Quadcopter Frame | Glass Fiber / Nylon Composites | Houses and isolates electronic components. |

---

## 🔌 Hardware Circuitry & Pin Layout

```
                        ESP32 Dev Module Pinout
                             ┌─────────┐
                         3V3 │         │ GND ◄──────────────────────┐
                       EN/RST│         │ GPIO 23                    │
                    Analog I2│         │ GPIO 22 ──► [MPU6050 SCL]  │
                    Analog I4│         │ GPIO 21 ──► [MPU6050 SDA]  │
                    Analog I3│         │ GPIO 19                    │
                    Analog I1│         │ GPIO 18                    │
                    Analog I0│         │ GPIO 17 ──► [LiDAR TX]     │
                    Analog I1│         │ GPIO 16 ──► [LiDAR RX]     │
                    Analog I2│         │ GPIO 4                     │
                    Analog I5│         │ GPIO 0                     │
                    Analog I6│         │ GPIO 2                     │
                    Analog I7│         │ GPIO 15                    │
                    Analog I8│         │ GPIO 14 ──► [ESC 3 - RL]   │
                         GND │         │ GPIO 12 ──► [ESC 2 - FR]   │
                         5V  │         │ GPIO 13 ──► [ESC 1 - FL]   │
                             └─────────┘
                                  │
                                  └────► GPIO 27 ──► [ESC 4 - RR]
```

### Complete System Connection Mapping

| Device Connection Source | ESP32 Target GPIO | Signal Type | Electrical Requirement / Safety Isolation |
| :--- | :--- | :--- | :--- |
| **MPU6050 SDA** | **GPIO 21** | I2C Data Line | Keep trace paths short to minimize interference. |
| **MPU6050 SCL** | **GPIO 22** | I2C Clock Line| Keep trace paths short to minimize interference. |
| **LiDAR Receiver (RX)** | **GPIO 16** | UART2 RX | Standard logic level communication matching. |
| **LiDAR Transmitter (TX)**| **GPIO 17** | UART2 TX | Standard logic level communication matching. |
| **ESC 1 — Front Left (FL)**| **GPIO 13** | LEDC PWM (CH0)| Assigns output steps to LEDC hardware channel 0. |
| **ESC 2 — Front Right (FR)**| **GPIO 12** | LEDC PWM (CH1)| Assigns output steps to LEDC hardware channel 1. |
| **ESC 3 — Rear Left (RL)** | **GPIO 14** | LEDC PWM (CH2)| Assigns output steps to LEDC hardware channel 2. |
| **ESC 4 — Rear Right (RR)**| **GPIO 27** | LEDC PWM (CH3)| Assigns output steps to LEDC hardware channel 3. |

---

## 🛑 Strict Bench Testing Safety Mandates

> 🔴 **CRITICAL SAFETY DIRECTIVE FOR DEVELOPERS:**
> * **PROPELLERS MUST BE REMOVED AT ALL TIMES** during bench assembly, code modifications, firmware upgrades, or peripheral testing operations. 2212 BLDC motors spin at over 10,000 RPM under load and can cause severe injury if triggered by a software bug or uncalibrated startup sequence.
> * **ESC CALIBRATION:** Ensure all four Electronic Speed Controllers are fully and uniformly calibrated to interpret minimum (1000 microseconds) and maximum (2000 microseconds) microsecond pulses accurately before connecting the flight battery pack.

---

## 📦 Software Libraries Required

Before compiling the firmware stack using the Arduino IDE, open the Library Manager (`Ctrl + Shift + I`) and install these standard dependencies:
1. **Adafruit MPU6050** (v2.2.0 or higher) - High-level abstraction driver for accessing raw IMU data.
2. **Adafruit Unified Sensor** (v1.1.4 or higher) - Unified base sensor class dependency.

---

## 🔧 Step-by-Step Installation & Compilation Guide

1. Clone or download this project source directly to your local computer.
2. Launch the Arduino IDE and open the main firmware code configuration file.
3. Select **Tools** -> **Board** -> **ESP32 Arduino** -> **ESP32 Dev Module**.
4. Configure the target board parameters precisely as follows:
   * **CPU Frequency:** 240MHz (Required for optimal multitasking scheduling)
   * **Core Debug Level:** None
   * **Flash Frequency:** 80MHz
5. Verify that the physical hardware lines match the system connection layout detailed above.
6. Remove the drone's propellers, connect the ESP32 via a high-speed micro-USB or Type-C cable, select the corresponding serial COM port, and click **Upload**.
7. Open the Serial Monitor (`115200 Baud`) to review initialization tracking logs, real-time sensor updates, and active FreeRTOS task assignment states.

---

