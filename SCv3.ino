#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

// Pins
const int trigPin = 13;
const int echoPin = 12;
const int servoPin = 11;
const int POWER_BUTTON_PIN = 2;

const int ESC_A_PIN = 3;
const int ESC_B_PIN = 6;
const int ESC_C_PIN = 5;
const int ESC_D_PIN = 9;

// Servo
Servo myServo;
const int SERVO_STOP = 91;
const int SERVO_CW   = 120;
const int SERVO_CCW  = 60;
const unsigned long SERVO_MOVE_TIME = 250;

// Ultrasonic
long duration;
int distance;
const int triggerDistance = 20;

// Handwave detection
int previousDistance = 0;
bool nearDetected = false;
bool triggered = false;
bool rotateCW = true;
const int waveIncrease = 20;

// MPU6050
MPU6050 mpu(Wire);
float initialRoll = 0;
float initialPitch = 0;

// ESCs
Servo escA, escB, escC, escD;

const int ESC_MAX = 2000;   // Full throttle CW
const int ESC_MIN = 1000;   // Full throttle CCW
const int ESC_STOP = 1500;  // Neutral / Stop
const int ESC_DEADBAND = 50; // 1450–1550

// Control tuning
const float KP = 3.0; // 10.0
const float DEADBAND = 5.0; // 2.5

// Power Button
bool systemOn = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Timer
unsigned long timer = 0;

// Function to apply ESC deadband
int applyDeadband(int signal)
{
  if(signal > ESC_STOP && signal < ESC_STOP + ESC_DEADBAND)
    signal = ESC_STOP + ESC_DEADBAND;

  if(signal < ESC_STOP && signal > ESC_STOP - ESC_DEADBAND)
    signal = ESC_STOP - ESC_DEADBAND;

  return constrain(signal, ESC_MIN, ESC_MAX);
}

// System Start
void systemStart() {

  Serial.println("Power ON: Initializing system...");

  previousDistance = 0;
  nearDetected = false;
  triggered = false;
  rotateCW = true;

  // MPU6050 init
  if (mpu.begin() != 0) {
    Serial.println("IMU not connected! Continuing without it.");
  } else {

    delay(1000);
    mpu.calcGyroOffsets();
    Serial.println("Calibration done!");

    delay(1000);

    mpu.update();
    initialRoll  = mpu.getAngleX();
    initialPitch = mpu.getAngleY();
  }

  // ESC Reset
  escA.writeMicroseconds(ESC_STOP);
  escB.writeMicroseconds(ESC_STOP);
  escC.writeMicroseconds(ESC_STOP);
  escD.writeMicroseconds(ESC_STOP);

  // Servo reset
  myServo.write(SERVO_STOP);

  delay(500);
}


// Setup
void setup() {

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);

  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);

  escA.attach(ESC_A_PIN);
  escB.attach(ESC_B_PIN);
  escC.attach(ESC_C_PIN);
  escD.attach(ESC_D_PIN);

  Serial.begin(115200);
  Wire.begin();

  Serial.println("System is OFF. Press button to power ON.");
}


// Main Loop
void loop() {

  // Button Handling
  int reading = digitalRead(POWER_BUTTON_PIN);

  if (reading != lastButtonState)
    lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading == LOW && !systemOn) {

      systemOn = true;
      systemStart();
      Serial.println("System Power ON");
    }

    else if (reading == LOW && systemOn) {

      systemOn = false;
      Serial.println("System Power OFF");

      myServo.write(SERVO_STOP);

      escA.writeMicroseconds(ESC_STOP);
      escB.writeMicroseconds(ESC_STOP);
      escC.writeMicroseconds(ESC_STOP);
      escD.writeMicroseconds(ESC_STOP);
    }
  }

  lastButtonState = reading;

  if (!systemOn) {
    delay(50);
    return;
  }


  // Ultrasonic Trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 100000);
  distance = duration * 0.034 / 2;


  // Handwave Detection
  if (distance > 0 && distance <= triggerDistance)
    nearDetected = true;

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

  if (distance > triggerDistance + 15)
    triggered = false;

  previousDistance = distance;


  // MPU Update
  mpu.update();

  float roll  = mpu.getAngleX();
  float pitch = mpu.getAngleY();

  float rollError = roll - initialRoll;
  float pitchError = pitch - initialPitch;

  // Deadband behavior
  // Smoother response instead of jumping immediately once out of deadband
  if (abs(rollError) < DEADBAND)
    rollError = 0;
  else
    rollError -= DEADBAND * (rollError > 0 ? 1 : -1);
  if (abs(pitchError) < DEADBAND)
    pitchError = 0;
  else
    pitchError -= DEADBAND * (pitchError > 0 ? 1 : -1);


  // Pitch Control
  int pitchCorrection = abs(pitchError) * KP;

  int pitchSignal = ESC_STOP;

  if (pitchError > 0)
    pitchSignal = ESC_STOP + pitchCorrection;

  else if (pitchError < 0)
    pitchSignal = ESC_STOP - pitchCorrection;

  pitchSignal = applyDeadband(pitchSignal);

  escA.writeMicroseconds(pitchSignal);
  escB.writeMicroseconds(pitchSignal);


  // Roll Control
  int rollCorrection = abs(rollError) * KP;

  int rollSignal = ESC_STOP;

  if (rollError > 0)
    rollSignal = ESC_STOP + rollCorrection;

  else if (rollError < 0)
    rollSignal = ESC_STOP - rollCorrection;

  rollSignal = applyDeadband(rollSignal);

  escC.writeMicroseconds(rollSignal);
  escD.writeMicroseconds(rollSignal);


  // Serial Output
  if (millis() - timer > 100) {

    Serial.print("Distance (cm): "); Serial.println(distance);
    // Serial.print(" | Roll: "); Serial.print(roll);
    // Serial.print(" | Pitch: "); Serial.println(pitch);
    timer = millis();
  }

  delay(20);
}
