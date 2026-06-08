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

## 🧑‍💻 Core Flight Control Code: `ESP32_Drone_Controller.ino`

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- PIN ASSIGNMENTS ---
#define IMU_SDA_PIN      21
#define IMU_SCL_PIN      22
#define LIDAR_RX_PIN     16
#define LIDAR_TX_PIN     17

#define MOTOR_FL_PIN     13
#define MOTOR_FR_PIN     12
#define MOTOR_RL_PIN     14
#define MOTOR_RR_PIN     27

// --- ESC PWM PARAMETERS ---
const int PWM_FREQ = 250;       // Standard multirotor ESC update frequency (250Hz - 400Hz)
const int PWM_RESOLUTION = 12;  // 12-bit resolution provides finer throttle control steps

// Channels mapping for LEDC PWM
const int CH_FL = 0;
const int CH_FR = 1;
const int CH_RL = 2;
const int CH_RR = 3;

// --- GLOBAL VARIABLES & SAFETY CONTROLS ---
Adafruit_MPU6050 mpu;
HardwareSerial LidarSerial(2); // Use UART2 for LiDAR data ingestion

// Volatile markers used across core tasks
volatile int globalLidarDistanceCm = 999; 
volatile bool activeSafetyOverride = false;
const int COLLISION_THRESHOLD_CM = 50; // Engage automatic braking at 50cm proximity

// Shared target baselines
volatile int baseThrottle = 1500; // Microseconds equivalent baseline placeholder

// --- FREERTOS TASK HANDLES ---
TaskHandle_t FlightCoreTask;
TaskHandle_t SafetyCoreTask;

// --- FUNCTION PROFILES ---
void writeMicrosecondsLEDC(int channel, int us) {
    // Converts a microsecond pulse duration (1000us - 2000us) into a matching 12-bit duty cycle step
    uint32_t duty = (us * PWM_FREQ * 4096) / 1000000;
    ledcWrite(channel, duty);
}

// =========================================================================
// CORE 0: PERIPHERAL INPUT & LI-DAR CRITICAL SAFETY INTERACTION
// =========================================================================
void SafetyCoreLoop(void * pvParameters) {
    Serial.print("Safety Core running on Core: ");
    Serial.println(xPortGetCoreID());

    uint8_t lidarBuffer[9];
    
    for(;;) {
        // Parse standard TF-Series or generic serial LiDAR data packages (9-byte protocol)
        if (LidarSerial.available() >= 9) {
            if (LidarSerial.read() == 0x59 && LidarSerial.read() == 0x59) { // Verify frame headers
                for (int i = 0; i < 7; i++) {
                    lidarBuffer[i] = LidarSerial.read();
                }
                
                // Low and high byte processing for raw distance values
                int currentDistance = lidarBuffer[0] + (lidarBuffer[1] << 8);
                
                if (currentDistance > 0) {
                    globalLidarDistanceCm = currentDistance;
                }
                
                // Check if the collision vector exceeds safe operating bounds
                if (globalLidarDistanceCm <= COLLISION_THRESHOLD_CM) {
                    activeSafetyOverride = true;
                } else {
                    activeSafetyOverride = false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield 10ms to keep system watchdogs clear
    }
}

// =========================================================================
// CORE 1: FLIGHT TRACKING, IMU DATA COLLECTION, & REAL-TIME PID LOOPS
// =========================================================================
void FlightCoreLoop(void * pvParameters) {
    Serial.print("Flight Core running on Core: ");
    Serial.println(xPortGetCoreID());

    // Basic PID tracking variables
    float rollError = 0, pitchError = 0;
    float rollPreviousError = 0, pitchPreviousError = 0;
    float Kp = 1.2, Kd = 0.05; // Loop tuning parameters
    
    sensors_event_t a, g, temp;

    for(;;) {
        mpu.getEvent(&a, &g, &temp);

        // Map raw acceleration data into degrees for alignment parsing
        float rollCurrent  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;
        float pitchCurrent = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;

        // Calculate stabilization corrections (Target orientation: 0 degrees flat)
        rollError = 0.0 - rollCurrent;
        pitchError = 0.0 - pitchCurrent;

        float rollCorrection  = (rollError * Kp) + ((rollError - rollPreviousError) * Kd);
        float pitchCorrection = (pitchError * Kp) + ((pitchError - pitchPreviousError) * Kd);

        rollPreviousError = rollError;
        pitchPreviousError = pitchError;

        // Establish core operational throttle parameters
        int operationalThrottle = baseThrottle;

        if (activeSafetyOverride) {
            // Apply automated obstacle mitigation strategy
            // Reduce overall power base and force a backwards pitch bias away from target object
            operationalThrottle = const_cast<int&>(baseThrottle) - 200; 
            pitchCorrection = pitchCorrection - 150; 
        }

        // Multirotor Mixer Equations (Quadcopter X-Configuration matching)
        int motorFL = operationalThrottle + pitchCorrection + rollCorrection;
        int motorFR = operationalThrottle + pitchCorrection - rollCorrection;
        int motorRL = operationalThrottle - pitchCorrection + rollCorrection;
        int motorRR = operationalThrottle - pitchCorrection - rollCorrection;

        // Enforce safe hardware operating boundaries (1000us minimum - 2000us maximum)
        motorFL = constrain(motorFL, 1000, 2000);
        motorFR = constrain(motorFR, 1000, 2000);
        motorRL = constrain(motorRL, 1000, 2000);
        motorRR = constrain(motorRR, 1000, 2000);

        // Update ESC outputs via high frequency PWM channels
        writeMicrosecondsLEDC(CH_FL, motorFL);
        writeMicrosecondsLEDC(CH_FR, motorFR);
        writeMicrosecondsLEDC(CH_RL, motorRL);
        writeMicrosecondsLEDC(CH_RR, motorRR);

        vTaskDelay(pdMS_TO_TICKS(4)); // Run loop at a high frequency cycle (~250Hz)
    }
}

// =========================================================================
// SYSTEM INITIALIZATION ROUTINES
// =========================================================================
void setup() {
    Serial.begin(115200);
    LidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    // Initialize MPU6050 Accelerometer
    if (!mpu.begin()) {
        Serial.println("MPU6050 configuration error. Halting startup.");
        while (1);
    }

    // Configure hardware LEDC peripherals for ESC control
    ledcSetup(CH_FL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_FR, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RL, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_RR, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(MOTOR_FL_PIN, CH_FL);
    ledcAttachPin(MOTOR_FR_PIN, CH_FR);
    ledcAttachPin(MOTOR_RL_PIN, CH_RL);
    ledcAttachPin(MOTOR_RR_PIN, CH_RR);

    // System initialization: Safe minimum pulse command out to the ESC array
    writeMicrosecondsLEDC(CH_FL, 1000);
    writeMicrosecondsLEDC(CH_FR, 1000);
    writeMicrosecondsLEDC(CH_RL, 1000);
    writeMicrosecondsLEDC(CH_RR, 1000);
    delay(2000); // Allow ESCs to parse start condition safely

    // Initialize the asynchronous core tasks
    xTaskCreatePinnedToCore(
        SafetyCoreLoop,      /* Functional block pointer */
        "SafetyCoreTask",    /* Dynamic name identifier */
        4096,                /* Stack scale allocation bytes */
        NULL,                /* Dynamic parameters array input */
        2,                   /* Priority index level */
        &SafetyCoreTask,     /* Output target pointer address tracking */
        0                    /* Map process to CPU Core 0 */
    );

    xTaskCreatePinnedToCore(
        FlightCoreLoop,
        "FlightCoreTask",
        4096,
        NULL,
        3,                   /* Flight critical controls assigned highest priority execution */
        &FlightCoreTask,
        1                    /* Map process to CPU Core 1 */
    );
}

void loop() {
    // Left unpopulated to preserve CPU performance metrics. 
    // All background processing runs inside the pinned Core loops.
    vTaskDelete(NULL);
}
```
