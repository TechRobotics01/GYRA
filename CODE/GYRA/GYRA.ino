#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <esp_task_wdt.h>

Adafruit_MPU6050 mpu;
Servo s2;

// ---------- PINS ----------
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int SERVO_PIN = 19;

// ---------- PID ----------
double input = 0;
double output = 0;
double setpoint = 0;

double Kp = 1.5, Ki = 0, Kd = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ---------- FILTER ----------
float angle = 0.0;
float gyroBiasY = 0.0;
unsigned long lastTime = 0;

// ---------- SERVO ----------
float servoSmooth = 90.0;

// ---------- WATCHDOG ----------
#define WDT_TIMEOUT_SEC 5

// ---------- SERIAL INPUT ----------
double readDoubleFromSerial(const char *prompt) {
  Serial.println(prompt);

  while (true) {
    while (Serial.available() == 0) {
      esp_task_wdt_reset();
      delay(10);
    }

    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.length() == 0) {
      Serial.println("Enter a valid number:");
      continue;
    }

    return s.toFloat();
  }
}

// ---------- CALIBRATION ----------
bool calibrateGyro() {
  Serial.println("Calibrating... KEEP STILL");

  const int targetSamples = 300;
  unsigned long start = millis();

  int count = 0;
  float sum = 0;

  while (count < targetSamples && (millis() - start < 10000)) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    if (!isnan(g.gyro.y)) {
      sum += g.gyro.y;
      count++;
    }

    if (count % 50 == 0 && count != 0) {
      Serial.print("Progress: ");
      Serial.print(count);
      Serial.println("/300");
    }

    esp_task_wdt_reset();
    delay(5);
  }

  if (count == 0) return false;

  gyroBiasY = sum / count;

  Serial.print("Gyro Bias: ");
  Serial.println(gyroBiasY, 6);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // ---------- WATCHDOG FIXED ----------
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  // ---------- I2C ----------
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

  // ---------- MPU ----------
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) {
      esp_task_wdt_reset();
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ---------- PID INPUT ----------
  Kp = readDoubleFromSerial("Enter Kp:");
  Serial.println(Kp, 6);

  Ki = readDoubleFromSerial("Enter Ki:");
  Serial.println(Ki, 6);

  Kd = readDoubleFromSerial("Enter Kd:");
  Serial.println(Kd, 6);

  myPID.SetTunings(Kp, Ki, Kd);

  // ---------- CALIBRATION ----------
  if (!calibrateGyro()) {
    Serial.println("Calibration failed!");
    while (1) {
      esp_task_wdt_reset();
      delay(1000);
    }
  }

  // ---------- SERVO ----------
  s2.attach(SERVO_PIN, 500, 2400);
  s2.write(90);
  delay(300);

  myPID.SetOutputLimits(-90, 90);
  myPID.SetMode(AUTOMATIC);

  lastTime = micros();
  Serial.println("READY");
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  if (dt <= 0 || dt > 0.05) dt = 0.01;

  float accAngle = atan2(-a.acceleration.x,
                         sqrt(a.acceleration.y * a.acceleration.y +
                              a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  float gyroRate = (g.gyro.y - gyroBiasY) * 180.0 / PI;

  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;

  if (isnan(angle)) {
    Serial.println("NaN error");
    esp_task_wdt_reset();
    return;
  }

  input = angle;
  myPID.Compute();

  float targetServo = 90 + output;

  servoSmooth = 0.75 * servoSmooth + 0.25 * targetServo;
  servoSmooth = constrain(servoSmooth, 0, 180);

  s2.write((int)servoSmooth);

  Serial.print("Angle: ");
  Serial.print(angle, 2);
  Serial.print(" Servo: ");
  Serial.println(servoSmooth, 1);

  esp_task_wdt_reset();
  delay(10);
}