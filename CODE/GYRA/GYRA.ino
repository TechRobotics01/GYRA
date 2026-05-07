#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <esp_task_wdt.h>

Adafruit_MPU6050 mpu;
Servo s2;   // Roll
Servo s3;   // Pitch
Servo s4;   // Yaw

// ---------- PINS ----------
const int SDA_PIN     = 21;
const int SCL_PIN     = 22;
const int SERVO_PINR  = 19;   // Roll
const int SERVO_PINP  = 18;   // Pitch
const int SERVO_PINY  = 5;    // Yaw  (GPIO 5 — LEDC-capable, safe for PWM)

// ---------- QMC5883P REGISTERS ----------
#define QMC5883P_ADDR   0x2C
#define REG_X_LSB       0x01   // Burst: X_L, X_H, Y_L, Y_H, Z_L, Z_H
#define REG_STATUS      0x09   // Bit 0 = DRDY
#define REG_MODE        0x0A   // 0xCF = Continuous, 200 Hz, OSR 512
#define REG_CONFIG      0x0B   // 0x08 = Set/Reset on, ±8 G range

// ---------- PID ROLL ----------
double input    = 0, output    = 0, setpoint    = 0;
double Kp = 1.2, Ki = 0, Kd = 0.1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ---------- PID PITCH ----------
double inputP   = 0, outputP   = 0, setpointP   = 0;
double KpP, KiP, KdP;
PID pidPitch(&inputP, &outputP, &setpointP, KpP, KiP, KdP, DIRECT);

// ---------- PID YAW ----------
double inputY   = 0, outputY   = 0, setpointY   = 0;
double KpY, KiY, KdY;
PID pidYaw(&inputY, &outputY, &setpointY, KpY, KiY, KdY, DIRECT);

// ---------- COMPLEMENTARY FILTER ----------
float angle       = 0.0;   // Roll  (degrees)
float pitch       = 0.0;   // Pitch (degrees)
float gyroBiasY   = 0.0;
float gyroBiasX   = 0.0;
unsigned long lastTime = 0;

// ---------- MAGNETOMETER CALIBRATION ----------
// Run the calibration sketch first, paste results here:
float mag_offset_x = 0.0f;
float mag_offset_y = 0.0f;

// ---------- YAW HEADING ----------
float yawHeading     = 0.0f;   // Current filtered heading (0–360°)
float yawHeadingFilt = 0.0f;   // Low-pass smoothed heading for PID input

// ---------- SERVO SMOOTHING ----------
float servoSmooth = 90.0;
float servoPitch  = 90.0;
float servoYaw    = 90.0;

// ---------- WATCHDOG ----------
#define WDT_TIMEOUT_SEC 5

// ═══════════════════════════════════════════════════════════════
//  UTILITIES
// ═══════════════════════════════════════════════════════════════

double readDoubleFromSerial(const char *prompt) {
  Serial.println(prompt);
  while (true) {
    while (Serial.available() == 0) {
      esp_task_wdt_reset();
      delay(10);
    }
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) { Serial.println("Enter a valid number:"); continue; }
    return s.toFloat();
  }
}

// ═══════════════════════════════════════════════════════════════
//  QMC5883P
// ═══════════════════════════════════════════════════════════════

void initQMC5883P() {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_MODE);
  Wire.write(0xCF);   // Continuous mode, 200 Hz ODR, OSR 512
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(0x08);   // Set/Reset enabled, ±8 G range
  Wire.endTransmission();
}

// Returns true when fresh data is available.
// x/y/z are raw signed 16-bit counts.
bool readQMC5883P(int16_t &x, int16_t &y, int16_t &z) {
  // Poll DRDY bit
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 1);
  if (!Wire.available()) return false;
  if (!(Wire.read() & 0x01)) return false;  // Not ready yet

  // Burst-read 6 bytes (little-endian: LSB first)
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_X_LSB);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 6);
  if (Wire.available() < 6) return false;

  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  uint8_t zl = Wire.read(), zh = Wire.read();

  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
  return true;
}

// Returns tilt-compensated heading in degrees [0, 360).
// Returns -1.0 if sensor data not ready.
float computeYawHeading(float roll_deg, float pitch_deg) {
  int16_t mx, my, mz;
  if (!readQMC5883P(mx, my, mz)) return -1.0f;

  // Hard-iron correction
  float fx = (float)mx - mag_offset_x;
  float fy = (float)my - mag_offset_y;
  float fz = (float)mz;

  // Tilt compensation using roll and pitch from complementary filter
  float roll_r  = roll_deg  * DEG_TO_RAD;
  float pitch_r = pitch_deg * DEG_TO_RAD;

  float cos_r = cosf(roll_r),  sin_r = sinf(roll_r);
  float cos_p = cosf(pitch_r), sin_p = sinf(pitch_r);

  float Xh = fx * cos_p + fz * sin_p;
  float Yh = fx * sin_r * sin_p + fy * cos_r - fz * sin_r * cos_p;

  float heading = atan2f(Yh, Xh) * RAD_TO_DEG;
  if (heading < 0.0f) heading += 360.0f;
  return heading;
}

// Shortest angular error, keeping result in (-180, +180].
// Correctly handles 0°↔360° wrap.
float yawError(float setpt, float measured) {
  float err = setpt - measured;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════════════

bool calibrateGyro() {
  Serial.println("Calibrating... KEEP STILL");
  const int targetSamples = 300;
  unsigned long start = millis();
  int count = 0;
  float sumY = 0, sumX = 0;

  while (count < targetSamples && (millis() - start < 10000)) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    if (!isnan(g.gyro.y) && !isnan(g.gyro.x)) {
      sumY += g.gyro.y;
      sumX += g.gyro.x;
      count++;
    }
    esp_task_wdt_reset();
    delay(5);
  }
  if (count == 0) return false;
  gyroBiasY = sumY / count;
  gyroBiasX = sumX / count;
  Serial.printf("Gyro Bias X: %.6f  Y: %.6f\n", gyroBiasX, gyroBiasY);
  return true;
}

// Capture hard-iron offsets for the magnetometer by rotating ~360°.
void calibrateMag() {
  Serial.println("MAG CAL: Rotate sensor slowly through 360° for 15 s, then hold still.");
  int16_t mx, my, mz;
  int16_t xmin =  32767, xmax = -32767;
  int16_t ymin =  32767, ymax = -32767;

  unsigned long start = millis();
  while (millis() - start < 15000) {
    if (readQMC5883P(mx, my, mz)) {
      if (mx < xmin) xmin = mx;
      if (mx > xmax) xmax = mx;
      if (my < ymin) ymin = my;
      if (my > ymax) ymax = my;
    }
    esp_task_wdt_reset();
    delay(20);
  }
  mag_offset_x = (xmax + xmin) / 2.0f;
  mag_offset_y = (ymax + ymin) / 2.0f;
  Serial.printf("Mag offsets  X: %.1f  Y: %.1f\n", mag_offset_x, mag_offset_y);
  Serial.println("Copy these values into mag_offset_x / mag_offset_y for permanent use.");
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(2000);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms   = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  // ---- MPU6050 ----
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) { esp_task_wdt_reset(); delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ---- QMC5883P ----
  initQMC5883P();
  delay(100);

  // ---- PID TUNING INPUT ----
  KpP = readDoubleFromSerial("Enter Pitch Kp:");  Serial.println(KpP, 6);
  KiP = readDoubleFromSerial("Enter Pitch Ki:");  Serial.println(KiP, 6);
  KdP = readDoubleFromSerial("Enter Pitch Kd:");  Serial.println(KdP, 6);
  pidPitch.SetTunings(KpP, KiP, KdP);

  KpY = readDoubleFromSerial("Enter Yaw Kp:");    Serial.println(KpY, 6);
  KiY = readDoubleFromSerial("Enter Yaw Ki:");    Serial.println(KiY, 6);
  KdY = readDoubleFromSerial("Enter Yaw Kd:");    Serial.println(KdY, 6);
  pidYaw.SetTunings(KpY, KiY, KdY);

  // ---- CALIBRATION ----
  if (!calibrateGyro()) {
    Serial.println("Gyro calibration failed!");
    while (1) { esp_task_wdt_reset(); delay(1000); }
  }

  // Comment this out once you have permanent offsets in mag_offset_x/y above.
  calibrateMag();

  // ---- SERVOS ----
  s2.attach(SERVO_PINR, 500, 2400);  s2.write(90);
  s3.attach(SERVO_PINP, 500, 2400);  s3.write(90);
  s4.attach(SERVO_PINY, 500, 2400);  s4.write(90);
  delay(300);

  // ---- PID LIMITS & MODE ----
  myPID.SetOutputLimits(-90, 90);
  pidPitch.SetOutputLimits(-90, 90);
  pidYaw.SetOutputLimits(-90, 90);

  myPID.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidYaw.SetMode(AUTOMATIC);

  // Seed the yaw setpoint from the initial heading so the gimbal
  // doesn't try to snap to 0° on first power-up.
  {
    int16_t mx, my, mz;
    // Wait for first valid reading
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
      if (readQMC5883P(mx, my, mz)) {
        float fx = (float)mx - mag_offset_x;
        float fy = (float)my - mag_offset_y;
        float h = atan2f(fy, fx) * RAD_TO_DEG;
        if (h < 0) h += 360.0f;
        setpointY       = h;
        yawHeadingFilt  = h;
        break;
      }
      esp_task_wdt_reset();
      delay(20);
    }
  }

  lastTime = micros();
  Serial.println("READY");
}

// ═══════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;

  // ---- ROLL (complementary filter) ----
  float accAngle = atan2f(-a.acceleration.x,
                          sqrtf(a.acceleration.y * a.acceleration.y +
                                a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
  float gyroRate = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;
  angle = 0.98f * (angle + gyroRate * dt) + 0.02f * accAngle;

  // ---- PITCH (complementary filter) ----
  float accPitch = atan2f(a.acceleration.y,
                          sqrtf(a.acceleration.x * a.acceleration.x +
                                a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
  float gyroPitch = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;
  pitch = 0.98f * (pitch + gyroPitch * dt) + 0.02f * accPitch;

  if (isnan(angle) || isnan(pitch)) {
    Serial.println("NaN error");
    esp_task_wdt_reset();
    return;
  }

  // ---- YAW (magnetometer — no gyro integration) ----
  float newHeading = computeYawHeading(angle, pitch);

  if (newHeading >= 0.0f) {
    // Low-pass filter the raw heading, handling the 0/360 wrap correctly.
    // We filter on the shortest-path delta from the current filtered value.
    float delta = newHeading - yawHeadingFilt;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    yawHeadingFilt += 0.15f * delta;   // α = 0.15  (tune: higher = faster response)

    if (yawHeadingFilt <   0.0f) yawHeadingFilt += 360.0f;
    if (yawHeadingFilt >= 360.0f) yawHeadingFilt -= 360.0f;
  }
  // If sensor wasn't ready this cycle, yawHeadingFilt holds its last value —
  // the PID simply repeats the previous correction, which is safe.

  // ---- YAW PID ----
  // PID_v1 computes (setpoint - input) internally but can't handle heading wrap.
  // We pre-compute the wrapped error and feed it as input relative to 0.
  float wrappedErr = yawError((float)setpointY, yawHeadingFilt);
  inputY    = -wrappedErr;   // PID computes (setpoint - input); with setpointY=0 → output ∝ +wrappedErr
  setpointY = 0;             // Temporarily zero so PID sees correct sign
  pidYaw.Compute();
  setpointY = yawHeadingFilt + wrappedErr;  // Restore true setpoint for next cycle

  // ---- ROLL PID ----
  input = angle;
  myPID.Compute();

  // ---- PITCH PID ----
  inputP = pitch;
  pidPitch.Compute();

  // ---- TARGET ANGLES ----
  float targetRoll  = 92.0f  + output;
  float targetPitch = 90.0f  + outputP;
  float targetYaw   = 90.0f  + outputY;

  // ---- SERVO SMOOTHING (same alpha as existing axes) ----
  servoSmooth = 0.75f * servoSmooth + 0.25f * targetRoll;
  servoPitch  = 0.75f * servoPitch  + 0.25f * targetPitch;
  servoYaw    = 0.75f * servoYaw    + 0.25f * targetYaw;

  servoSmooth = constrain(servoSmooth, 0, 180);
  servoPitch  = constrain(servoPitch,  0, 180);
  servoYaw    = constrain(servoYaw,    0, 180);

  s2.write((int)servoSmooth);
  s3.write((int)servoPitch);
  s4.write((int)servoYaw);

  // ---- TELEMETRY ----
  Serial.print("R:");   Serial.print(angle,          2);
  Serial.print(" P:");  Serial.print(pitch,           2);
  Serial.print(" Y:");  Serial.print(yawHeadingFilt,  1);
  Serial.print(" SR:"); Serial.print(servoSmooth);
  Serial.print(" SP:"); Serial.print(servoPitch);
  Serial.print(" SY:"); Serial.println(servoYaw);

  esp_task_wdt_reset();
  delay(10);
}
