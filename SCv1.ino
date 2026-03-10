#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

// Ultrasonic + Servo
const int trigPin = 13;
const int echoPin = 12;
const int servoPin = 11;

Servo myServo;

long duration;
int distance;

bool triggered = false;
bool rotateCW = true;

// Continuous-rotation servo values
const int SERVO_STOP = 91;
const int SERVO_CW   = 120;
const int SERVO_CCW  = 60;

const int triggerDistance = 15; // cm
const unsigned long SERVO_MOVE_TIME = 400; // ms

// Wave detection variables
int previousDistance = 0;
bool nearDetected = false;
const int waveIncrease = 100; // cm increase required to count as wave

// MPU6050
MPU6050 mpu(Wire);
unsigned long timer = 0;

float initialRoll = 0;
float initialPitch = 0;

// ESCs
// A & B for Pitch
// C & D for Roll
Servo escA;
Servo escB;
Servo escC;
Servo escD;

const int ESC_A_PIN = 3;
const int ESC_B_PIN = 10;
const int ESC_C_PIN = 5;
const int ESC_D_PIN = 6;

const int ESC_MIN = 1500;
const int ESC_MAX = 1900;
const int ESC_MIN_REV = 1100;

// Control Tuning
const float KP = 10.0;
const float DEADBAND = 2.5;

void setup() {

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);

  escA.attach(ESC_A_PIN);
  escB.attach(ESC_B_PIN);
  escC.attach(ESC_C_PIN);
  escD.attach(ESC_D_PIN);

  escA.writeMicroseconds(ESC_MIN);
  escB.writeMicroseconds(ESC_MIN);
  escC.writeMicroseconds(ESC_MIN);
  escD.writeMicroseconds(ESC_MIN);

  Serial.begin(115200);
  Wire.begin();

  if (mpu.begin() != 0) {
    Serial.println("IMU not connected! Continuing without it.");
  } 
  else {
    Serial.println("IMU connected, calibrating...");
    delay(1000);
    mpu.calcGyroOffsets();
    Serial.println("Calibration done!");
    delay(1000);
    mpu.update();
    initialRoll  = mpu.getAngleX();
    initialPitch = mpu.getAngleY();
  }

  Serial.print("Initial Roll: ");
  Serial.println(initialRoll);
  Serial.print("Initial Pitch: ");
  Serial.println(initialPitch);

  delay(2000);
}

void loop() {

  // Ultrasonic Measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
  distance = duration * 0.034 / 2;

  /* ---------- Wave Trigger Logic ---------- */

  // Detect when object comes close
  if (distance > 0 && distance <= triggerDistance) {
    nearDetected = true;
  }

  // Detect quick move away
  if (nearDetected && (distance - previousDistance) > waveIncrease && !triggered) {

    triggered = true;
    nearDetected = false;

    if (rotateCW) {
      myServo.write(SERVO_CW);
      delay(SERVO_MOVE_TIME);
      myServo.write(SERVO_STOP);
      rotateCW = false;
    } 
    else {
      myServo.write(SERVO_CCW);
      delay(SERVO_MOVE_TIME);
      myServo.write(SERVO_STOP);
      rotateCW = true;
    }
  }

  // Reset system once object is far away
  if (distance > triggerDistance + 15) {
    triggered = false;
  }
  previousDistance = distance;

  // MPU6050 Update
  mpu.update();

  float roll  = mpu.getAngleX();
  float pitch = mpu.getAngleY();

  float rollError  = roll  - initialRoll;
  float pitchError = pitch - initialPitch;

  if (abs(rollError) < DEADBAND) rollError = 0;
  if (abs(pitchError) < DEADBAND) pitchError = 0;

  // Pitch Control
  int pitchCorrection = abs(pitchError) * KP;
  pitchCorrection = constrain(pitchCorrection, 0, ESC_MAX - ESC_MIN);

  int pitchSignal;

  if (pitchError > 0) {
    pitchSignal = ESC_MIN + pitchCorrection;
  } 
  else if (pitchError < 0) {
    pitchSignal = ESC_MIN - pitchCorrection;
  } 
  else {
    pitchSignal = ESC_MIN;
  }

  escA.writeMicroseconds(pitchSignal);
  escB.writeMicroseconds(pitchSignal);

  // Roll Control
  int rollCorrection = abs(rollError) * KP;
  rollCorrection = constrain(rollCorrection, 0, ESC_MAX - ESC_MIN);

  int rollSignal;

  if (rollError > 0) {
    rollSignal = ESC_MIN + rollCorrection;
  } 
  else if (rollError < 0) {
    rollSignal = ESC_MIN - rollCorrection;
  } 
  else {
    rollSignal = ESC_MIN;
  }

  escC.writeMicroseconds(rollSignal);
  escD.writeMicroseconds(rollSignal);

  // Serial Output
  if (millis() - timer > 100) {
    Serial.print("Distance (cm): ");
    Serial.print(distance);
    Serial.print(" | Roll: ");
    Serial.print(roll);
    Serial.print(" | Pitch: ");
    Serial.println(pitch);
    timer = millis();
  }
  delay(20);
}
