#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <esp_task_wdt.h>

Adafruit_MPU6050 mpu;
Servo s2;
Servo s3;

// ---------- PINS ----------
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int SERVO_PINR = 19;
const int SERVO_PINP = 18;

// ---------- PID ROLL ----------
double input = 0;
double output = 0;
double setpoint = 0;

double Kp = 1.2, Ki = 0, Kd = 0.1;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ---------- PID PITCH ----------
double inputP = 0;
double outputP = 0;
double setpointP = 0;
double KpP, KiP, KdP;
PID pidPitch(&inputP, &outputP, &setpointP, KpP, KiP, KdP, DIRECT);

// ---------- FILTER ----------
float angle = 0.0;
float pitch = 0.0;
float gyroBiasY = 0.0;
float gyroBiasX = 0.0;
unsigned long lastTime = 0;

// ---------- SERVO ----------
float servoSmooth = 90.0;
float servoPitch = 90.0;

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
  float sumY = 0;
  float sumX = 0;

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

  Serial.print("Gyro Bias Y: ");
  Serial.println(gyroBiasY, 6);
  Serial.print("Gyro Bias X: ");
  Serial.println(gyroBiasX, 6);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };

  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50);

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

  // ---------- PITCH PID INPUT ----------
  KpP = readDoubleFromSerial("Enter Pitch Kp:");
  Serial.println(KpP, 6);

  KiP = readDoubleFromSerial("Enter Pitch Ki:");
  Serial.println(KiP, 6);

  KdP = readDoubleFromSerial("Enter Pitch Kd:");
  Serial.println(KdP, 6);

  pidPitch.SetTunings(KpP, KiP, KdP);

  // ---------- CALIBRATION ----------
  if (!calibrateGyro()) {
    Serial.println("Calibration failed!");
    while (1) {
      esp_task_wdt_reset();
      delay(1000);
    }
  }

  // ---------- SERVO ----------
  s2.attach(SERVO_PINR, 500, 2400);
  s2.write(90);
  s3.attach(SERVO_PINP, 500, 2400);
  s3.write(90);
  delay(300);

  myPID.SetOutputLimits(-90, 90);
  pidPitch.SetOutputLimits(-90, 90);

  myPID.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);

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

  // ---------- ROLL ----------
  float accAngle = atan2(-a.acceleration.x,
                         sqrt(a.acceleration.y * a.acceleration.y +
                              a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  float gyroRate = (g.gyro.y - gyroBiasY) * 180.0 / PI;

  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;

  // ---------- PITCH (FILTERED NOW) ----------
  float accPitch = atan2(a.acceleration.y,
                         sqrt(a.acceleration.x * a.acceleration.x +
                              a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  float gyroPitch = (g.gyro.x - gyroBiasX) * 180.0 / PI;

  pitch = 0.98 * (pitch + gyroPitch * dt) + 0.02 * accPitch;

  if (isnan(angle) || isnan(pitch)) {
    Serial.println("NaN error");
    esp_task_wdt_reset();
    return;
  }

  // ---------- PID ----------
  input = angle;
  myPID.Compute();

  inputP = pitch;
  pidPitch.Compute();

  float targetRoll = 90 + output;
  float targetPitch = 90 + outputP;

  // ---------- SMOOTH ----------
  servoSmooth = 0.75 * servoSmooth + 0.25 * targetRoll;
  servoPitch = 0.75 * servoPitch + 0.25 * targetPitch;

  servoSmooth = constrain(servoSmooth, 0, 180);
  servoPitch = constrain(servoPitch, 0, 180);

  s2.write((int)servoSmooth);
  s3.write((int)servoPitch);

  Serial.print("R:");
  Serial.print(angle, 2);
  Serial.print(" P:");
  Serial.print(pitch, 2);
  Serial.print(" SR:");
  Serial.print(servoSmooth);
  Serial.print(" SP:");
  Serial.println(servoPitch);

  esp_task_wdt_reset();
  delay(10);
}