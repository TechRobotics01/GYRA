#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>

const int servo1 = 25;
const int servo2 = 26;

Servo s1;
Servo s2;

unsigned long prevTime = 0;
float yaw = 0;

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_ax, *mpu_gr;

void setup() {
  Serial.begin(115200);

  s1.attach(servo1);
  s2.attach(servo2);

  Serial.println("WELCOME TO GYRA ");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  Serial.println("MPU6050 FOUND!");

  mpu_ax = mpu.getAccelerometerSensor();
  mpu_gr = mpu.getGyroSensor();

  mpu_ax->printSensorDetails();
  mpu_gr->printSensorDetails();
}

void loop() {
  sensors_event_t ax;
  sensors_event_t gr;

  mpu_ax->getEvent(&ax);
  mpu_gr->getEvent(&gr);

  float ax_val = ax.acceleration.x;
  float ay_val = ax.acceleration.y;
  float az_val = ax.acceleration.z;

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float roll = atan2(ay_val, az_val) * 180 / PI;

  float gz = gr.gyro.z;
  yaw += gz * dt * 180 / PI;

  yaw = map((int)yaw, 0, 360, 0, 255);
  roll = map((int)roll, 0, 360, 0, 255);

  Serial.print("ROLL: ");
  Serial.print(roll);
  Serial.print(" YAW: ");
  Serial.println(yaw);

  s1.write(yaw);
  s2.write(roll);

  delay(10);
}