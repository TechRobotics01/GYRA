#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu;
Servo s2;

// ---------- PID ----------
double input = 0;
double output = 0;
double setpoint = 0;

double Kp = 1.2, Ki = 1.6, Kd = 0.6;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ---------- FILTER ----------
float angle = 0.0;
float gyroBiasY = 0.0;
unsigned long lastTime = 0;

// ---------- SERVO ----------
float servoSmooth = 90.0;
const int servoPin = 19;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  s2.attach(servoPin, 500, 2400);
  s2.write(90);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 🔧 Gyro calibration (KEEP STILL)
  Serial.println("Calibrating...");
  delay(1000);

  float sum = 0;
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.y;
    delay(5);
  }
  gyroBiasY = sum / 300.0;

  myPID.SetOutputLimits(-90, 90);
  myPID.SetMode(AUTOMATIC);

  lastTime = micros();
  Serial.println("READY (PITCH AXIS)");
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // ----- TIME -----
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  if (dt <= 0 || dt > 0.05) dt = 0.01;

  // 🔥 CORRECT PITCH AXIS (FORWARD/BACK)
  float accAngle = atan2(-a.acceleration.x,
                         sqrt(a.acceleration.y * a.acceleration.y +
                              a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  float gyroRate = (g.gyro.y - gyroBiasY) * 180.0 / PI;

  // ----- COMPLEMENTARY FILTER -----
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;

  input = angle;

  // ----- PID -----
  myPID.Compute();

  // 🔥 SERVO COMMAND
  float targetServo = 90.0 + output;

  // smoothing
  servoSmooth = 0.75 * servoSmooth + 0.25 * targetServo;
  servoSmooth = constrain(servoSmooth, 0, 180);

  s2.write((int)servoSmooth);

  // debug
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" Servo: ");
  Serial.println(servoSmooth);

  delay(10);
}