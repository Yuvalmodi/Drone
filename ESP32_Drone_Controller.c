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
