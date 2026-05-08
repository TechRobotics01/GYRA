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
const int SDA_PIN    = 21;
const int SCL_PIN    = 22;
const int SERVO_PINR = 19;
const int SERVO_PINP = 18;
const int SERVO_PINY = 5;

// ---------- QMC5883P ----------
#define QMC5883P_ADDR  0x2C
#define REG_X_LSB      0x01
#define REG_STATUS     0x09
#define REG_MODE       0x0A
#define REG_CONFIG     0x0B

// ---------- PID ROLL ----------
// FIX: explicitly initialise all tuning doubles to 0.0
//      PID_v1 constructor runs at global-init time; if the doubles are
//      uninitialised the internal pointer/value arithmetic is undefined
//      behaviour and causes the null-deref crash (EXCCAUSE 7, EXCVADDR 0).
double input = 0, output = 0, setpoint = 0;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ---------- PID PITCH ----------
double inputP = 0, outputP = 0, setpointP = 0;
double KpP = 0.0, KiP = 0.0, KdP = 0.0;
PID pidPitch(&inputP, &outputP, &setpointP, KpP, KiP, KdP, DIRECT);

// ---------- PID YAW ----------
double inputY = 0, outputY = 0, setpointY = 0;   // setpointY stays 0 always
double KpY = 0.0, KiY = 0.0, KdY = 0.0;
PID pidYaw(&inputY, &outputY, &setpointY, KpY, KiY, KdY, DIRECT);

// ---------- FILTER ----------
float angle = 0.0;
float pitch = 0.0;
float gyroBiasY = 0.0;
float gyroBiasX = 0.0;
unsigned long lastTime = 0;

// ---------- MAG CALIBRATION ----------
float mag_offset_x = 0.0f;
float mag_offset_y = 0.0f;

// ---------- YAW ----------
float yawHeadingFilt  = 0.0f;   // low-pass filtered current heading
float yawLockedHeading = 0.0f;  // FIX: the heading we want to hold —
                                 // kept here, NOT in setpointY, so it
                                 // is never accidentally overwritten in loop()

// ---------- SERVO SMOOTH ----------
float servoSmooth = 90.0;
float servoPitch  = 90.0;
float servoYaw    = 90.0;

// ---------- WATCHDOG ----------
#define WDT_TIMEOUT_SEC 5

// ═══════════════════════════════════════
// SAFE SERIAL INPUT
// ═══════════════════════════════════════

double readDoubleFromSerial(const char *prompt) {
  Serial.println(prompt);
  while (true) {
    esp_task_wdt_reset();
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() == 0) {
        Serial.println("Enter valid number:");
        continue;
      }
      return s.toFloat();
    }
    delay(1);
  }
}

// ═══════════════════════════════════════
// QMC5883P
// ═══════════════════════════════════════

void initQMC5883P() {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_MODE);
  Wire.write(0xCF);   // Continuous, 200 Hz, OSR 512
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(0x08);   // Set/Reset on, ±8 G
  Wire.endTransmission();
}

bool readQMC5883P(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(REG_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 1);
  if (!Wire.available()) return false;
  if (!(Wire.read() & 0x01)) return false;

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

float computeYawHeading(float roll_deg, float pitch_deg) {
  int16_t mx, my, mz;
  if (!readQMC5883P(mx, my, mz)) return -1.0f;

  float fx = (float)mx - mag_offset_x;
  float fy = (float)my - mag_offset_y;
  float fz = (float)mz;

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

// Shortest angular error in (-180, +180]
float yawError(float locked, float measured) {
  float err = locked - measured;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// ═══════════════════════════════════════
// CALIBRATION
// ═══════════════════════════════════════

void calibrateGyro() {
  Serial.println("KEEP STILL");
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
  Serial.printf("Bias X: %.6f  Y: %.6f\n", gyroBiasX, gyroBiasY);
}

void calibrateMag() {
  Serial.println("MAG CAL: rotate 360 degrees slowly for 15 s");
  int16_t mx, my, mz;
  int16_t xmin =  32767, xmax = -32768;
  int16_t ymin =  32767, ymax = -32768;
  unsigned long end = millis() + 15000;
  while (millis() < end) {
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
  Serial.printf("mag_offset_x = %.1f\nmag_offset_y = %.1f\n",
                mag_offset_x, mag_offset_y);
  Serial.println("Hardcode these above and remove calibrateMag() call.");
}

// ═══════════════════════════════════════
// SETUP
// ═══════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(2000);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);   // FIX: prevents I2C bus hang outlasting the watchdog

  if (!mpu.begin()) {
    Serial.println("MPU6050 FAILED");
    while (1) { esp_task_wdt_reset(); delay(1000); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  initQMC5883P();

  // ── PID tuning ──────────────────────────────
  Kp = readDoubleFromSerial("Enter Roll Kp:");
  Ki = readDoubleFromSerial("Enter Roll Ki:");
  Kd = readDoubleFromSerial("Enter Roll Kd:");
  myPID.SetTunings(Kp, Ki, Kd);

  KpP = readDoubleFromSerial("Enter Pitch Kp:");
  KiP = readDoubleFromSerial("Enter Pitch Ki:");
  KdP = readDoubleFromSerial("Enter Pitch Kd:");
  pidPitch.SetTunings(KpP, KiP, KdP);

  KpY = readDoubleFromSerial("Enter Yaw Kp:");
  KiY = readDoubleFromSerial("Enter Yaw Ki:");
  KdY = readDoubleFromSerial("Enter Yaw Kd:");
  pidYaw.SetTunings(KpY, KiY, KdY);

  // ── Calibrate ───────────────────────────────
  calibrateGyro();

  // Comment out after pasting offsets into mag_offset_x/y above:
  calibrateMag();

  // ── Servos ──────────────────────────────────
  s2.attach(SERVO_PINR, 500, 2400);
  s3.attach(SERVO_PINP, 500, 2400);
  s4.attach(SERVO_PINY, 500, 2400);
  s2.write(90); s3.write(90); s4.write(90);
  delay(300);

  myPID.SetOutputLimits(-90, 90);
  pidPitch.SetOutputLimits(-90, 90);
  pidYaw.SetOutputLimits(-90, 90);

  myPID.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidYaw.SetMode(AUTOMATIC);

  // ── Seed yaw lock from initial heading ──────
  // Wait up to 2 s for the first valid magnetometer reading so the
  // gimbal doesn't try to swing to 0° (North) on first power-up.
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
      int16_t mx, my, mz;
      if (readQMC5883P(mx, my, mz)) {
        float fx = (float)mx - mag_offset_x;
        float fy = (float)my - mag_offset_y;
        float h  = atan2f(fy, fx) * RAD_TO_DEG;
        if (h < 0.0f) h += 360.0f;
        yawLockedHeading = h;
        yawHeadingFilt   = h;
        break;
      }
      esp_task_wdt_reset();
      delay(20);
    }
  }

  lastTime = micros();
  Serial.println("READY");
}

// ═══════════════════════════════════════
// LOOP
// ═══════════════════════════════════════

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;

  // ---------- ROLL ----------
  float accAngle = atan2f(
                     -a.acceleration.x,
                     sqrtf(a.acceleration.y * a.acceleration.y +
                           a.acceleration.z * a.acceleration.z)
                   ) * RAD_TO_DEG;
  float gyroRate = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;
  angle = 0.98f * (angle + gyroRate * dt) + 0.02f * accAngle;

  // ---------- PITCH ----------
  float accPitch = atan2f(
                     a.acceleration.y,
                     sqrtf(a.acceleration.x * a.acceleration.x +
                           a.acceleration.z * a.acceleration.z)
                   ) * RAD_TO_DEG;
  float gyroPitch = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;
  pitch = 0.98f * (pitch + gyroPitch * dt) + 0.02f * accPitch;

  // ---------- YAW ----------
  float newHeading = computeYawHeading(angle, pitch);
  if (newHeading >= 0.0f) {
    float delta = newHeading - yawHeadingFilt;
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    yawHeadingFilt += 0.15f * delta;
    if (yawHeadingFilt <   0.0f) yawHeadingFilt += 360.0f;
    if (yawHeadingFilt >= 360.0f) yawHeadingFilt -= 360.0f;
  }

  // ---------- PID ----------
  input  = angle;
  myPID.Compute();

  inputP = pitch;
  pidPitch.Compute();

  // FIX: setpointY is permanently 0; yawLockedHeading holds the target.
  // inputY = -(locked - measured) so PID output = +correction when we've
  // rotated away from the locked heading.
  inputY    = -yawError(yawLockedHeading, yawHeadingFilt);
  setpointY = 0.0;   // never changes
  pidYaw.Compute();

  // ---------- TARGETS ----------
  float targetRoll  = 92.0f + output;
  float targetPitch = 90.0f + outputP;
  float targetYaw   = 90.0f + outputY;

  // ---------- SMOOTH ----------
  servoSmooth = 0.75f * servoSmooth + 0.25f * targetRoll;
  servoPitch  = 0.75f * servoPitch  + 0.25f * targetPitch;
  servoYaw    = 0.75f * servoYaw    + 0.25f * targetYaw;

  servoSmooth = constrain(servoSmooth, 0, 180);
  servoPitch  = constrain(servoPitch,  0, 180);
  servoYaw    = constrain(servoYaw,    0, 180);

  // ---------- SERVOS ----------
  s2.write((int)servoSmooth);
  s3.write((int)servoPitch);
  s4.write((int)servoYaw);

  // ---------- TELEMETRY ----------
  Serial.print("R:");  Serial.print(angle,          2);
  Serial.print(" P:"); Serial.print(pitch,           2);
  Serial.print(" Y:"); Serial.print(yawHeadingFilt,  1);
  Serial.print(" L:"); Serial.print(yawLockedHeading,1);
  Serial.print(" SR:"); Serial.print(servoSmooth);
  Serial.print(" SP:"); Serial.print(servoPitch);
  Serial.print(" SY:"); Serial.println(servoYaw);

  esp_task_wdt_reset();
  delay(10);
}
