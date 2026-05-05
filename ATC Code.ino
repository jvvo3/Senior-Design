#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_DPS310.h>

// Pins
const int trigPin = 13;
const int echoPin = 12;
const int servoPin = 11;
const int POWER_BUTTON_PIN = 2;

const int ESC_A_PIN = 5;
const int ESC_B_PIN = 3;
const int ESC_C_PIN = 9;
const int ESC_D_PIN = 6;

// Servo
Servo myServo;
const int SERVO_STOP = 90;
const int SERVO_CW   = 110;
const int SERVO_CCW  = 85;

// Ultrasonic
long duration;
int distance;
const int triggerDistance = 15;

// Handwave detection
int previousDistance = 0;
bool nearDetected = false;
bool triggered = false;
bool rotateCW = true;
const int waveIncrease = 20;
bool servoMoving = false;
unsigned long servoStartTime = 0;
unsigned long servoDuration = 0;

// MPU6050 (IMU)
MPU6050 mpu(Wire);
float initialRoll = 0;
float initialPitch = 0;

// DPS310 (Altitude)
Adafruit_DPS310 dps;
#define SEA_LEVEL_HPA 1007.00
#define SMOOTH_SAMPLES 10
float altReadings[SMOOTH_SAMPLES];
int altReadIndex = 0;
float altTotal = 0;
float smoothedAltitude = 0;

// ESCs
Servo escA, escB, escC, escD;

const int ESC_MAX = 1700;
const int ESC_MIN = 1300;
const int ESC_STOP = 1500;
const int ESC_DEADBAND = 50;

// Control tuning
const float KP = 17.0; 
const float DEADBAND = 1.5;

float filteredRoll = 0;
float filteredPitch = 0;
const float ANGLE_ALPHA = 0.7;

// Power Button
bool systemOn = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Timer
unsigned long timer = 0;


int applyDeadband(int signal)
{
  if (abs(signal - ESC_STOP) < ESC_DEADBAND)
    return ESC_STOP;

  return constrain(signal, ESC_MIN, ESC_MAX);
}

void rampMotor(Servo& esc, int from, int to, int stepDelay) {
  int step = (to > from) ? 1 : -1;
  for (int s = from; s != to + step; s += step) {
    esc.writeMicroseconds(s);
    delay(stepDelay);
  }
}

void startupSequence() {
  Serial.println("Running startup sequence...");

  rampMotor(escA, ESC_STOP, 1570, 10); rampMotor(escA, 1570, ESC_STOP, 10);
  rampMotor(escC, ESC_STOP, 1570, 10); rampMotor(escC, 1570, ESC_STOP, 10); 
  rampMotor(escB, ESC_STOP, 1570, 10); rampMotor(escB, 1570, ESC_STOP, 10); 
  rampMotor(escD, ESC_STOP, 1570, 10); rampMotor(escD, 1570, ESC_STOP, 10); 

  Serial.println("Startup sequence complete.");
}

void shutdownSequence() {
  Serial.println("Running shutdown sequence...");

  rampMotor(escA, ESC_STOP, 1570, 10); rampMotor(escA, 1570, ESC_STOP, 10); 
  rampMotor(escC, ESC_STOP, 1570, 10); rampMotor(escC, 1570, ESC_STOP, 10); 
  rampMotor(escB, ESC_STOP, 1570, 10); rampMotor(escB, 1570, ESC_STOP, 10);
  rampMotor(escD, ESC_STOP, 1570, 10); rampMotor(escD, 1570, ESC_STOP, 10); 

  Serial.println("Shutdown sequence complete.");
}

void initialSequence() {

  for (int s = ESC_STOP; s <= 1570; s++) {
    escA.writeMicroseconds(s);
    escB.writeMicroseconds(s);
    escC.writeMicroseconds(s);
    escD.writeMicroseconds(s);
    delay(10);
  }

  for (int s = 1570; s >= ESC_STOP; s--) {
    escA.writeMicroseconds(s);
    escB.writeMicroseconds(s);
    escC.writeMicroseconds(s);
    escD.writeMicroseconds(s);
    delay(10);
  }

  Serial.println("Initializing sequence complete.");
}

void initDPS310() {
  if (!dps.begin_I2C(0x76)) {
    Serial.println("DPS310 not found! Check wiring.");
    return;
  }

  dps.configurePressure(DPS310_128HZ, DPS310_2SAMPLES);
  dps.configureTemperature(DPS310_128HZ, DPS310_2SAMPLES);

  delay(200);
  sensors_event_t t, p;
  dps.getEvents(&t, &p);
  float init = 44330.0 * (1.0 - pow(p.pressure / SEA_LEVEL_HPA, 0.1903));
  for (int i = 0; i < SMOOTH_SAMPLES; i++) altReadings[i] = init;
  altTotal = init * SMOOTH_SAMPLES;
  smoothedAltitude = init;
}

void systemStart() {

  Serial.println("Power ON: Initializing system...");

  previousDistance = 0;
  nearDetected = false;
  triggered = false;
  rotateCW = true;

  if (mpu.begin() != 0) {
    Serial.println("IMU not connected! Continuing without it.");
  } else {
    mpu.calcGyroOffsets();

    unsigned long start = millis();
    while (millis() - start < 3000) {
      mpu.update();
      delay(5);
    }

    const int SETTLE_SAMPLES = 400;
    float rollSum = 0;
    float pitchSum = 0;

    for (int i = 0; i < SETTLE_SAMPLES; i++) {
      mpu.update();
      rollSum  += mpu.getAngleX();
      pitchSum += mpu.getAngleY();
      delay(20);
    }

    initialRoll  = (rollSum  / SETTLE_SAMPLES) - 180.0;
    initialPitch = (pitchSum / SETTLE_SAMPLES);
    filteredRoll  = initialRoll;
    filteredPitch = initialPitch;

    Serial.print("Initial Roll: "); Serial.print(initialRoll);
    Serial.print(" | Initial Pitch: "); Serial.println(initialPitch);
  }

  initDPS310();

  escA.writeMicroseconds(ESC_STOP);
  escB.writeMicroseconds(ESC_STOP);
  escC.writeMicroseconds(ESC_STOP);
  escD.writeMicroseconds(ESC_STOP);

  myServo.write(SERVO_STOP);
  delay(500);

  initialSequence();
}

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

void loop() {
  // Push Button Power
  int reading = digitalRead(POWER_BUTTON_PIN);

  if (reading != lastButtonState)
    lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading == LOW && !systemOn) {
      systemOn = true;
      startupSequence();
      systemStart();
    }

    else if (reading == LOW && systemOn) {
      systemOn = false;
      shutdownSequence();

      myServo.write(SERVO_STOP);
      escA.writeMicroseconds(ESC_STOP);
      escB.writeMicroseconds(ESC_STOP);
      escC.writeMicroseconds(ESC_STOP);
      escD.writeMicroseconds(ESC_STOP);

      Serial.println("System Power OFF");
    }
  }

  lastButtonState = reading;

  if (!systemOn) {
    delay(50);
    return;
  }

  // Ultrasonic and Servo Logic
  static unsigned long lastUltra = 0;
  if (millis() - lastUltra >= 50) {
      lastUltra = millis();

      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      long d = pulseIn(echoPin, HIGH, 30000);
      if (d > 0) {
          distance = d * 0.034 / 2;
      } else {
          distance = 999;
      }
  }

  if (distance > 0 && distance <= triggerDistance)
    nearDetected = true;

  if (nearDetected && (distance - previousDistance) > waveIncrease && !triggered) {
    triggered = true;
    nearDetected = false;
    if (rotateCW) {
      myServo.write(SERVO_CW);
      servoDuration = 490;
      rotateCW = false;
    } else {
      myServo.write(SERVO_CCW);
      servoDuration = 350;
      rotateCW = true;
    }
    servoMoving = true;
    servoStartTime = millis();
  }

  if (distance > triggerDistance + 15)
    triggered = false;

  previousDistance = distance;

  if (servoMoving && (millis() - servoStartTime >= servoDuration)) {
    myServo.write(SERVO_STOP);
    servoMoving = false;
  }

  mpu.update();

  float roll  = mpu.getAngleX();
  float pitch = mpu.getAngleY();

  float unwrappedRoll = roll - 180.0;
  if (unwrappedRoll < -180.0) unwrappedRoll += 360.0;

  filteredRoll  = ANGLE_ALPHA * filteredRoll  + (1 - ANGLE_ALPHA) * unwrappedRoll;
  filteredPitch = ANGLE_ALPHA * filteredPitch + (1 - ANGLE_ALPHA) * pitch;

  float rollError  = -(filteredRoll  - initialRoll);
  float pitchError = filteredPitch - initialPitch;

  if (abs(rollError) < DEADBAND)  rollError = 0;
  if (abs(pitchError) < DEADBAND) pitchError = 0;

  // PD calculation
  float rollOutput  = KP * rollError;
  float pitchOutput = -(KP * pitchError);

  // Constrain output
  rollOutput  = constrain(rollOutput,  -200, 200);
  pitchOutput = constrain(pitchOutput, -200, 200);

  // Differential thrust
  int escC_signal = applyDeadband(ESC_STOP + rollOutput);
  int escD_signal = applyDeadband(ESC_STOP + rollOutput);

  int escA_signal = applyDeadband(ESC_STOP - pitchOutput);
  int escB_signal = applyDeadband(ESC_STOP - pitchOutput);

  escC.writeMicroseconds(escC_signal);
  escD.writeMicroseconds(escD_signal);
  escA.writeMicroseconds(escA_signal);
  escB.writeMicroseconds(escB_signal);

  // SERIAL
  sensors_event_t temp_event, pressure_event;
  dps.getEvents(&temp_event, &pressure_event);

  float altitude = 44330.0 * (1.0 - pow(pressure_event.pressure / SEA_LEVEL_HPA, 0.1903));

  altTotal -= altReadings[altReadIndex];
  altReadings[altReadIndex] = altitude;
  altTotal += altitude;
  altReadIndex = (altReadIndex + 1) % SMOOTH_SAMPLES;
  smoothedAltitude = altTotal / SMOOTH_SAMPLES;

  if (millis() - timer > 100) {
    Serial.print("Roll: "); Serial.print(unwrappedRoll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Distance: "); Serial.print(distance);
    Serial.print(" | Altitude: "); Serial.println(smoothedAltitude);
    timer = millis();
  }
}
