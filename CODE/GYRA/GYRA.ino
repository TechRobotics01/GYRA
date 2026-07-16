#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <cstdarg>
#include <math.h>

Adafruit_MPU6050 mpu; 

// ---------- PINS ----------
const int SDA_PIN    = 8;
const int SCL_PIN    = 9;

const int SERVO_PINR = 5; // Roll Servo Pin
const int SERVO_PINP = 6; // Pitch Servo Pin

// ---------- PID ROLL ----------
double input = 0, output = 0, setpoint = 0;
double Kp = 1.2;  // Activated and matched to Pitch for instant testing
double Ki = 0.0;
double Kd = 0.1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ---------- PID PITCH ----------
double inputP = 0, outputP = 0, setpointP = 0;
double KpP = 1.2;
double KiP = 0.0;
double KdP = 0.1;
PID pidPitch(&inputP, &outputP, &setpointP, KpP, KiP, KdP, DIRECT);

// ---------- FILTER ----------
float angle = 0.0;
float pitch = 0.0;
float gyroBiasY = 0.0;
float gyroBiasX = 0.0;
unsigned long lastTime = 0;

// ---------- SERVO SMOOTHING ----------
float servoSmooth = 90.0;
float servoPitch  = 90.0;

// ---------- WATCHDOG ----------
#define WDT_TIMEOUT_SEC 5

// ═══════════════════════════════════════
// DEBUG & LOGGING SYSTEM
// ═══════════════════════════════════════
enum DebugLevel {
    LEVEL_NONE,
    LEVEL_ERROR,
    LEVEL_WARNING,
    LEVEL_INFO,
    LEVEL_VERBOSE
};

DebugLevel currentLogLevel = LEVEL_VERBOSE; 

#define ANSI_RESET   "\033[0m"
#define ANSI_RED     "\033[31m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_CYAN    "\033[36m"

void debugPrint(DebugLevel level, const char *format, ...) {
    if (level > currentLogLevel) return;
    
    switch (level) {
        case LEVEL_ERROR:   Serial.print(ANSI_RED "[ERROR] " ANSI_RESET); break;
        case LEVEL_WARNING: Serial.print(ANSI_YELLOW "[WARNING] " ANSI_RESET); break;
        case LEVEL_INFO:    Serial.print(ANSI_GREEN "[INFO] " ANSI_RESET); break;
        case LEVEL_VERBOSE: Serial.print(ANSI_CYAN "[VERBOSE] " ANSI_RESET); break;
        default: break;
    }
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    Serial.println(buffer);
}

#define DEBUG_ERROR(fmt, ...)   debugPrint(LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define DEBUG_WARNING(fmt, ...) debugPrint(LEVEL_WARNING, fmt, ##__VA_ARGS__)
#define DEBUG_INFO(fmt, ...)    debugPrint(LEVEL_INFO, fmt, ##__VA_ARGS__)
#define DEBUG_VERBOSE(fmt, ...) debugPrint(LEVEL_VERBOSE, fmt, ##__VA_ARGS__)

// ═══════════════════════════════════════
// NATIVE SERVO DRIVER
// ═══════════════════════════════════════
void setServoAngle(int pin, int angle) {
  // Maps 0-180 degrees to standard 500us - 2400us pulses using 12-bit PWM (0-4095) at 50Hz
  int duty = map(constrain(angle, 0, 180), 0, 180, 102, 491);
  analogWrite(pin, duty);
}

// ═══════════════════════════════════════
// TIMING & DIAGNOSTICS GLOBALS
// ═══════════════════════════════════════
unsigned long loopTimeMin = 999999;
unsigned long loopTimeMax = 0;
unsigned long loopTimeSum = 0;
unsigned long loopCount   = 0;

unsigned long lastLoopStatsTimer = 0;
unsigned long lastMemTimer       = 0;
unsigned long lastHealthTimer    = 0;
unsigned long lastTelemetryTimer = 0;
unsigned long lastSatWarning     = 0;

// ═══════════════════════════════════════
// CRASH DIAGNOSTICS
// ═══════════════════════════════════════
void printCrashDiagnostics() {
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason != ESP_RST_POWERON && reason != ESP_RST_SW && reason != ESP_RST_EXT) {
        Serial.println(ANSI_RED "\n===========================");
        Serial.println("    SYSTEM FAULT DETECTED");
        Serial.println("===========================" ANSI_RESET);
        Serial.printf("Reason Code: %d\n", reason);
        Serial.printf("Free Heap: %u Bytes\n", ESP.getFreeHeap());
        Serial.println("Attempting Recovery...\n");
        delay(1000);
    }
}

// ═══════════════════════════════════════
// GYRO CALIBRATION
// ═══════════════════════════════════════
void calibrateGyro() {
  DEBUG_INFO("Keeping still for Gyroscope Calibration...");
  const int N = 300;
  float sumY = 0, sumX = 0;
  
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sumY += g.gyro.y;
    sumX += g.gyro.x;
    esp_task_wdt_reset();
    delay(5);
  }
  gyroBiasY = sumY / N;
  gyroBiasX = sumX / N;
  Serial.println("GYRO CALIBRATION COMPLETE\n");
}

// ═══════════════════════════════════════
// I2C RECOVERY
// ═══════════════════════════════════════
void recoverI2CBus() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);   delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);  delayMicroseconds(5);
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);     delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);    delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);    delayMicroseconds(5);
}

// ═══════════════════════════════════════
// SETUP
// ═══════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  printCrashDiagnostics();

  Serial.println("\n===========================");
  Serial.println(" GYRA 2-AXIS STARTUP DIAG");
  Serial.println("===========================");
  
  Serial.printf("ESP32 Boot..............PASS\n");
  Serial.printf("Free Heap...............%u Bytes\n\n", ESP.getFreeHeap());

  // Configure Watchdog
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic  = true
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);

  Serial.print("Running I2C Recovery....");
  recoverI2CBus();
  Serial.println("PASS");

  Serial.print("Initializing I2C........");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  delay(100);
  Serial.println("PASS\n");

  Serial.print("Initializing MPU6050...");
  bool mpuReady = false;
  int retryCount = 0;
  while (!mpuReady && retryCount < 6) {
    if (mpu.begin(0x68, &Wire)) {
      mpuReady = true;
    } else {
      retryCount++;
      Serial.print(".");
      esp_task_wdt_reset();
      delay(500);
    }
  }

  if (!mpuReady) {
    Serial.println("FAILED");
    while (1) { esp_task_wdt_reset(); delay(1000); }
  }
  Serial.println("PASS");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  myPID.SetTunings(Kp, Ki, Kd);
  pidPitch.SetTunings(KpP, KiP, KdP);

  calibrateGyro();

  // Configure Native PWM Servos for Roll and Pitch
  Serial.print("Configuring Native PWM Servos...");
  analogWriteResolution(SERVO_PINR, 12); analogWriteFrequency(SERVO_PINR, 50);
  analogWriteResolution(SERVO_PINP, 12); analogWriteFrequency(SERVO_PINP, 50);
  Serial.println("PASS");
  
  // Park servos at center
  setServoAngle(SERVO_PINR, 90); 
  setServoAngle(SERVO_PINP, 90); 
  delay(300);

  myPID.SetOutputLimits(-90, 90);
  pidPitch.SetOutputLimits(-90, 90);

  myPID.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);

  Serial.println("\n===========================");
  DEBUG_INFO("2-Axis System Ready.");
  Serial.println("===========================\n");
  
  lastTime = micros();
  unsigned long mNow = millis();
  lastLoopStatsTimer = mNow;
  lastMemTimer       = mNow;
  lastHealthTimer    = mNow;
  lastTelemetryTimer = mNow;
}

// ═══════════════════════════════════════
// NaN & Bounds Protection 
// ═══════════════════════════════════════
bool isDataValid(sensors_event_t& a, sensors_event_t& g) {
    if (isnan(a.acceleration.x) || isinf(a.acceleration.x)) return false;
    if (isnan(a.acceleration.y) || isinf(a.acceleration.y)) return false;
    if (isnan(a.acceleration.z) || isinf(a.acceleration.z)) return false;
    if (isnan(g.gyro.x) || isinf(g.gyro.x)) return false;
    if (isnan(g.gyro.y) || isinf(g.gyro.y)) return false;
    if (isnan(g.gyro.z) || isinf(g.gyro.z)) return false;
    if (fabs(a.acceleration.x) > 50.0 || fabs(a.acceleration.y) > 50.0 || fabs(a.acceleration.z) > 50.0) return false;
    if (fabs(g.gyro.x) > 12.0 || fabs(g.gyro.y) > 12.0 || fabs(g.gyro.z) > 12.0) return false;
    return true;
}

// ═══════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════
void loop() {
  unsigned long loopStartMicros = micros();
  unsigned long currentMillis   = millis();

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  
  if (!isDataValid(a, g)) {
      DEBUG_ERROR("Sensor Fault Detected / Skipping Data Frame");
      esp_task_wdt_reset();
      delay(10);
      return;
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;

  // ---------- ROLL CALCULATION ----------
  float accAngle = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
  float gyroRate = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;
  angle = 0.98f * (angle + gyroRate * dt) + 0.02f * accAngle;

  // ---------- PITCH CALCULATION ----------
  float accPitch = atan2f(a.acceleration.y, sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
  float gyroPitch = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;
  pitch = 0.98f * (pitch + gyroPitch * dt) + 0.02f * accPitch;

  // ---------- PID COMPUTE ----------
  input  = angle; myPID.Compute();
  inputP = pitch; pidPitch.Compute();

  // ---------- TARGET GENERATION ----------
  float targetRoll  = 92.0f + output;
  float targetPitch = 90.0f - outputP;

  // ---------- LOW-PASS SERVO SMOOTHING ----------
  servoSmooth = 0.75f * servoSmooth + 0.25f * targetRoll;
  servoPitch  = 0.75f * servoPitch  + 0.25f * targetPitch;

  servoSmooth = constrain(servoSmooth, 0, 180);
  servoPitch  = constrain(servoPitch,  0, 180);

  // ---------- NATIVE SERVO WRITE ----------
  setServoAngle(SERVO_PINR, (int)servoSmooth);
  setServoAngle(SERVO_PINP, (int)servoPitch);

  // ---------- LOOP TIMING PROFILER ----------
  unsigned long loopDuration = micros() - loopStartMicros;
  if (loopDuration < loopTimeMin) loopTimeMin = loopDuration;
  if (loopDuration > loopTimeMax) loopTimeMax = loopDuration;
  loopTimeSum += loopDuration;
  loopCount++;

  // ═══════════════════════════════════════
  // SCHEDULED DIAGNOSTICS & TELEMETRY
  // ═══════════════════════════════════════
  if (currentMillis - lastSatWarning > 500) {
      bool sat = false;
      if (output >= 90.0 || output <= -90.0)   { DEBUG_WARNING("Roll Saturated"); sat = true; }
      if (outputP >= 90.0 || outputP <= -90.0) { DEBUG_WARNING("Pitch Saturated"); sat = true; }
      if (sat) lastSatWarning = currentMillis;
  }

  if (currentMillis - lastHealthTimer >= 1000) {
      lastHealthTimer = currentMillis;
      Wire.beginTransmission(0x68);
      if (Wire.endTransmission() != 0) mpu.begin();
  }

  if (currentMillis - lastLoopStatsTimer >= 1000) {
      lastLoopStatsTimer = currentMillis;
      if (loopCount > 0) {
          float avgTime = (float)loopTimeSum / loopCount;
          Serial.println("\n[LOOP TIMING]");
          Serial.printf("  Average: %.2f us\n  Minimum: %lu us\n  Maximum: %lu us\n", avgTime, loopTimeMin, loopTimeMax);
      }
      loopTimeMin = 999999; loopTimeMax = 0; loopTimeSum = 0; loopCount = 0;
  }

  if (currentMillis - lastMemTimer >= 5000) {
      lastMemTimer = currentMillis;
      Serial.println("\n[MEMORY MONITOR]");
      Serial.printf("  Free Heap: %u Bytes\n", ESP.getFreeHeap());
  }

  if (currentMillis - lastTelemetryTimer >= 250) {
      lastTelemetryTimer = currentMillis;
      if (currentLogLevel >= LEVEL_INFO) {
          Serial.println("\n--- TELEMETRY ---");
          Serial.printf("Roll  | Angle: %5.2f | PID Output: %5.2f | Servo Target: %5.2f\n", angle, output, servoSmooth);
          Serial.printf("Pitch | Angle: %5.2f | PID Output: %5.2f | Servo Target: %5.2f\n", pitch, outputP, servoPitch);
      }
  }

  esp_task_wdt_reset();
  delay(10);
}