#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

// ---------- MPU6050 ----------
MPU6050 mpu(Wire);

// ---------- SERVOS ----------
Servo servoRoll;
Servo servoPitch;

// ---------- PINS ----------
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const int ROLL_SERVO_PIN  = 19;
const int PITCH_SERVO_PIN = 18;

// ---------- PID ROLL ----------
double rollInput = 0;
double rollOutput = 0;
double rollSetpoint = 0;

double KpR = 1.2;
double KiR = 0;
double KdR = 0.1;

PID pidRoll(
  &rollInput,
  &rollOutput,
  &rollSetpoint,
  KpR,
  KiR,
  KdR,
  DIRECT
);

// ---------- PID PITCH ----------
double pitchInput = 0;
double pitchOutput = 0;
double pitchSetpoint = 0;

double KpP;
double KiP;
double KdP;

PID pidPitch(
  &pitchInput,
  &pitchOutput,
  &pitchSetpoint,
  KpP,
  KiP,
  KdP,
  DIRECT
);

// ---------- SMOOTHING ----------
float smoothRoll  = 90;
float smoothPitch = 90;

// ---------- SERIAL INPUT ----------
double readDoubleFromSerial(
  const char *prompt
) {

  Serial.println(prompt);

  while (Serial.available() == 0) {
    delay(10);
  }

  String s =
    Serial.readStringUntil('\n');

  s.trim();

  return s.toFloat();
}

void setup() {

  Serial.begin(115200);

  delay(2000);

  // ---------- I2C ----------
  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);

  // ---------- MPU6050 ----------
  mpu.begin();

  Serial.println("KEEP MPU6050 STILL");

  delay(1000);

  // ---------- GYRO CAL ----------
  mpu.calcGyroOffsets(true);

  Serial.println("GYRO CALIBRATION DONE");

  // ---------- PITCH PID ----------
  KpP =
    readDoubleFromSerial(
      "Enter Pitch Kp:"
    );

  KiP =
    readDoubleFromSerial(
      "Enter Pitch Ki:"
    );

  KdP =
    readDoubleFromSerial(
      "Enter Pitch Kd:"
    );

  pidPitch.SetTunings(
    KpP,
    KiP,
    KdP
  );

  // ---------- SERVOS ----------
  servoRoll.attach(
    ROLL_SERVO_PIN,
    500,
    2400
  );

  servoPitch.attach(
    PITCH_SERVO_PIN,
    500,
    2400
  );

  servoRoll.write(90);
  servoPitch.write(90);

  delay(1000);

  // ---------- PID ----------
  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);

  pidRoll.SetOutputLimits(-90, 90);
  pidPitch.SetOutputLimits(-90, 90);

  Serial.println("GIMBAL READY");
}

void loop() {

  // ---------- UPDATE MPU ----------
  mpu.update();

  // ---------- GET ANGLES ----------
  float rollAngle =
    mpu.getAngleX();

  float pitchAngle =
    mpu.getAngleY();

  // ---------- PID INPUT ----------
  rollInput = rollAngle;
  pitchInput = pitchAngle;

  // ---------- COMPUTE PID ----------
  pidRoll.Compute();
  pidPitch.Compute();

  // ---------- TARGETS ----------
  float targetRoll =
    90 + rollOutput;

  float targetPitch =
    90 + pitchOutput;

  // ---------- SMOOTHING ----------
  smoothRoll =
    0.75 * smoothRoll
    +
    0.25 * targetRoll;

  smoothPitch =
    0.75 * smoothPitch
    +
    0.25 * targetPitch;

  // ---------- LIMITS ----------
  smoothRoll =
    constrain(
      smoothRoll,
      0,
      180
    );

  smoothPitch =
    constrain(
      smoothPitch,
      0,
      180
    );

  // ---------- MOVE SERVOS ----------
  servoRoll.write(
    (int)smoothRoll
  );

  servoPitch.write(
    (int)smoothPitch
  );

  // ---------- DEBUG ----------
  Serial.print("ROLL: ");
  Serial.print(rollAngle);

  Serial.print(" PITCH: ");
  Serial.print(pitchAngle);

  Serial.print(" SR: ");
  Serial.print(smoothRoll);

  Serial.print(" SP: ");
  Serial.println(smoothPitch);

  delay(10);
}