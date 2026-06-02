#include <Servo.h>

// ===== STEPPER 1 =====
const int dirPin1      = 2;
const int stepPin1     = 3;
const int enablePin1   = 8;
const int limitSwitch1 = 7;

// ===== STEPPER 2 =====
const int dirPin2      = 6;
const int stepPin2     = 4;
const int enablePin2   = 10;
const int limitSwitch2 = 11;

// ===== GANTRY SERVO =====
const int gantryServoPin    = 9;
const int limitSwitchGantry = A2;

// ===== GRIPPER SERVO =====
const int gripperServoPin    = 5;
const int limitSwitchGripper = A1;

// ===== CUTTING SERVO =====
const int cuttingServoPin = 13;

// ===== PUMP =====
const int pumpPin = 12;

// ===== PRESSURE SENSOR =====
const int pressureSensor = A0;

// ===== TRIGGER BUTTON =====
const int triggerPin = A4;

// ===== STEPPER SETTINGS =====
const float stepsPerRevolution = 400.0; // steps per revolution (half-step A4988)
const int moveDelay1 = 300;             // us per step — gantry stepper speed
const int moveDelay2 = 400;             // us per step — substrate gripper speed
const int slowDelay  = 500;             // us per step — homing speed (slower for accuracy)
const int safeOffset = 80;              // steps to back off from limit switch after homing

// ===== USED PLUG CALIBRATION =====
const float plug_stepper1_downRounds = 10.2;                        // rounds down to reach used substrate
const float plug_stepper2_diglength  = 3.75;                         // physical dig length in cm
const float plug_stepper2_digRounds  = plug_stepper2_diglength / 0.8; // converted to motor rounds

// ===== NEW PLUG & SEED CALIBRATION =====
const float stepper1_firstDownRounds  = 10.0;                      // rounds down to pick new substrate
const float stepper1_secondDownRounds = 10.0;                      // rounds down to place new substrate
const float stepper1_upRounds         = 3.0;                       // rounds up after seed pickup before moving
const float stepper1_thirdDownRounds  = 5.0;                       // rounds down to deposit seed
const float stepper2_downLength       = 2;                         // dig lenth for 
const float stepper2_downRounds       = stepper2_downLength / 0.8; // rounds for substrate gripper on new plug

// ===== CUTTING SERVO CALIBRATION =====
const int cuttingStartAngle = 30;  // resting/home angle for root cutter
const int cuttingEndAngle   = 150; // cutting stroke end angle
const int cuttingPauseMs    = 500; // ms to hold at each end of stroke

// ===== GANTRY SERVO CALIBRATION =====
const int gantryHomingSpeed   = 10;  // ms per degree during homing sweep
const int gantrySafeOffset    = 23;  // degrees to back off after switch triggers
const int gantryHomeAngle     = 23;  // startup write angle before homing
const int gantryRestAngle     = 0;   // rest/home position relative to zero
const int gantryDiscardAngle  = 160; // position for discarding used plug
const int gantrySeedPick      = 90;  // position for seed deposit
const int gantrySubstratePick = 160; // position for picking new substrate
const int gantryMinDelay      = 2;   // ms per degree at peak speed 
const int gantryMaxDelay      = 10;  // ms per degree at start and end of sweep

// ===== GRIPPER SERVO CALIBRATION =====
const int gripperHomingSpeed = 15; // ms per degree during homing
const int gripperSafeOffset  = 5;  // degrees back from switch = zero reference
const int gripperOpenAngle   = 80; // degrees from zero = fully open
const int gripperGripAngle   = 25; // degrees from zero = gripping position

// ===== PRESSURE CALIBRATION =====
const float V_MIN                   = 0.1727; // sensor zero-pressure voltage offset
const float V_RANGE                 = 4.5;    // sensor full-scale voltage range
const float P_MAX                   = 500.0;  // sensor full-scale pressure in kPa
const float pressureClogLimit       = 40.0;   // kPa — clog test must stay BELOW this
const float pressurePickedThreshold = 50.0;   // kPa — must exceed to confirm seed picked
const float pressureDropThreshold   = 50.0;   // kPa — must stay above while pump running
const float pressureAlpha           = 0.1;    // EMA smoothing factor (0=no update, 1=no smoothing)
const int   clogTestDuration        = 5000;   // ms to run pump during clog test

// ===== SEED PICKUP RETRY =====
const int          seedPickupRetries = 2;     // extra attempts after first failure
const unsigned long pickupWait       = 4000;  // ms to wait stationary before oscillating
const unsigned long pickupMax        = 10000; // ms total per attempt before giving up

// ===== OBJECTS =====
Servo gantryServo;
Servo gripperServo;
Servo cuttingServo;

// ===== STATE =====
bool stepper1Homed     = false;
bool stepper2Homed     = false;
bool pumpActive        = false;
int  gantryZero        = 0;   // absolute angle where gantry switch triggered + offset
int  gripperZero       = 0;   // absolute angle where gripper switch triggered - offset
float pressureSmoothed = 0;   // running EMA of raw pressure ADC reading

// Step position tracking — counts steps below home, always >= 0
long stepper1Pos = 0;
long stepper2Pos = 0;

// ===== HELPERS =====

// Converts float revolution count to integer step count for A4988 driver
long roundsToSteps(float rounds) {
  return (long)(rounds * stepsPerRevolution);
}

// Prevents servo write values from going outside 0-180 degree hardware limit
int clampAngle(int angle) {
  if (angle < 0)   return 0;
  if (angle > 180) return 180;
  return angle;
}

// Issues a single step pulse to the A4988 driver on the given step pin
void stepMotor(int sPin, int delayVal) {
  digitalWrite(sPin, HIGH);
  delayMicroseconds(delayVal);
  digitalWrite(sPin, LOW);
  delayMicroseconds(delayVal);
}

// Moves a stepper a fixed number of steps without position tracking or switch checking.
void moveStepsRaw(int sPin, int dPin, long steps, bool dirUp, int delayVal, int enPin) {
  if (steps <= 0) return;
  digitalWrite(enPin, LOW);
  digitalWrite(dPin, dirUp ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    stepMotor(sPin, delayVal);
  }
  digitalWrite(enPin, HIGH);
}

// Moves stepper 1 by the given number of steps while tracking absolute position.
void moveStepper1(long steps, bool dirUp) {
  if (steps <= 0) return;
  digitalWrite(enablePin1, LOW);
  digitalWrite(dirPin1, dirUp ? HIGH : LOW);

  for (long i = 0; i < steps; i++) {
    if (dirUp && digitalRead(limitSwitch1) == LOW) {
      digitalWrite(enablePin1, HIGH);
      Serial.println("ERROR: Stepper 1 hit limit switch unexpectedly. Check mechanism.");
      moveStepsRaw(stepPin1, dirPin1, safeOffset, false, slowDelay, enablePin1);
      stepper1Pos = safeOffset;
      return;
    }
    stepMotor(stepPin1, moveDelay1);
    stepper1Pos += dirUp ? -1 : 1;
    if (stepper1Pos < 0) stepper1Pos = 0;
  }

  digitalWrite(enablePin1, HIGH);
}

// Moves stepper 2 by the given number of steps while tracking absolute position.
void moveStepper2(long steps, bool dirUp) {
  if (steps <= 0) return;
  digitalWrite(enablePin2, LOW);
  digitalWrite(dirPin2, dirUp ? HIGH : LOW);

  for (long i = 0; i < steps; i++) {
    if (dirUp && digitalRead(limitSwitch2) == LOW) {
      digitalWrite(enablePin2, HIGH);
      Serial.println("ERROR: Stepper 2 hit limit switch unexpectedly. Check mechanism.");
      moveStepsRaw(stepPin2, dirPin2, safeOffset, false, slowDelay, enablePin2);
      stepper2Pos = safeOffset;
      return;
    }
    stepMotor(stepPin2, moveDelay2);
    stepper2Pos += dirUp ? -1 : 1;
    if (stepper2Pos < 0) stepper2Pos = 0;
  }

  digitalWrite(enablePin2, HIGH);
}

// Returns stepper 1 exactly as many steps upward as it currently is below home, then resets position tracker to zero. Guarantees no overshoot.
void returnStepper1() {
  if (stepper1Pos <= 0) return;
  moveStepper1(stepper1Pos, true);
  stepper1Pos = 0;
}

// Returns stepper 2 exactly as many steps upward as it currently is below home, then resets position tracker to zero. Guarantees no overshoot.
void returnStepper2() {
  if (stepper2Pos <= 0) return;
  moveStepper2(stepper2Pos, true);
  stepper2Pos = 0;
}

// Drives stepper toward limit switch, waits for trigger, backs off safeOffset steps, and sets position tracker to safeOffset 
void homeStepper(int sPin, int dPin, int sw, int enPin, bool &homedFlag, long &posTracker) {
  digitalWrite(enPin, LOW);
  if (digitalRead(sw) == LOW) {
    digitalWrite(dPin, LOW);
    while (digitalRead(sw) == LOW) stepMotor(sPin, slowDelay);
  }
  digitalWrite(dPin, HIGH);
  while (digitalRead(sw) == HIGH) stepMotor(sPin, slowDelay);
  digitalWrite(enPin, HIGH);
  moveStepsRaw(sPin, dPin, safeOffset, false, slowDelay, enPin);
  posTracker = safeOffset;
  homedFlag = true;
}


// Blocks execution until the physical trigger button is pressed and released.
void waitForPress(const char* message) {
  Serial.println(message);
  while (digitalRead(triggerPin) == LOW);
  while (digitalRead(triggerPin) == HIGH);
  delay(50);
}

// ===== PRESSURE =====

// Converts smoothed ADC voltage to absolute pressure in kPa
float voltageToPressure(float voltage) {
  return ((voltage - V_MIN) / V_RANGE) * P_MAX;
}

// Reads raw ADC, applies exponential moving average smoothing
float readPressure() {
  int raw = analogRead(pressureSensor);
  pressureSmoothed = pressureAlpha * raw + (1.0 - pressureAlpha) * pressureSmoothed;
  float voltage = pressureSmoothed * (5.0 / 1023.0);
  return voltageToPressure(voltage);
}

// Runs pump for clogTestDuration ms and monitors pressure.

void runClogTest() {
  pressureSmoothed = analogRead(pressureSensor);
  digitalWrite(pumpPin, HIGH);

  unsigned long start = millis();
  float maxP = 0;

  while (millis() - start < (unsigned long)clogTestDuration) {
    float p = readPressure();
    if (p > maxP) maxP = p;

    if (p > pressureClogLimit) {
      digitalWrite(pumpPin, LOW);
      Serial.print("ERROR: Clog detected at ");
      Serial.print(p);
      Serial.println(" kPa. Clear blockage and reset.");
      while (true);
    }
    delay(200);
  }

  digitalWrite(pumpPin, LOW);
}

// Drives gantry servo toward decreasing angle until limit switch triggers
void homeGantry() {
  int angle = gantryServo.read();

  while (digitalRead(limitSwitchGantry) == HIGH) {
    angle--;
    if (angle < 0) {
      angle = 0;
      gantryServo.write(0);
      Serial.println("ERROR: Gantry switch not found. Check wiring and reset.");
      while (true);
    }
    gantryServo.write(angle);
    delay(gantryHomingSpeed);
  }

  angle = clampAngle(angle + gantrySafeOffset);
  gantryServo.write(angle);
  delay(300);
  gantryZero = angle;
}

// Moves gantry servo to a position expressed as degrees from gantryZero.
void moveGantryTo(int degreesFromZero) {
  int target  = clampAngle(gantryZero + degreesFromZero);
  int current = gantryServo.read();
  int steps   = abs(target - current);
  if (steps == 0) return;
  int dir = (target > current) ? 1 : -1;

  for (int i = 0; i < steps; i++) {
    current += dir;
    gantryServo.write(current);
    float progress = (float)i / (steps - 1);
    int d = (int)(gantryMinDelay + (gantryMaxDelay - gantryMinDelay) * (1.0 - sin(PI * progress)));
    delay(d);
  }
}

// ===== GRIPPER SERVO =====

// Drives gripper servo toward increasing angle until limit switch triggers
void homeGripper() {
  int angle = gripperServo.read();

  while (digitalRead(limitSwitchGripper) == HIGH) {
    angle++;
    if (angle > 180) {
      angle = 180;
      gripperServo.write(angle);
      Serial.println("ERROR: Gripper switch not found. Check wiring and reset.");
      while (true);
    }
    gripperServo.write(angle);
    delay(gripperHomingSpeed);
  }

  angle = clampAngle(angle - gripperSafeOffset);
  gripperServo.write(angle);
  delay(300);
  gripperZero = angle;
}

// Moves gripper to a position expressed as degrees subtracted from gripperZero.
void moveGripperTo(int degreesFromZero, int msPerDegree = 10) {
  int target  = clampAngle(gripperZero - degreesFromZero);
  int current = gripperServo.read();
  int step    = (target > current) ? 1 : -1;

  while (current != target) {
    current += step;
    gripperServo.write(clampAngle(current));
    delay(msPerDegree);
  }
}

// ===== CUTTING SERVO =====

// Executes five full cutting strokes
void performCut() {
  cuttingServo.write(cuttingStartAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingEndAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingEndAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle);
  cuttingServo.write(cuttingEndAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle);
  cuttingServo.write(cuttingEndAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle);
  cuttingServo.write(cuttingEndAngle);
  delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle);
}

// Called on any unrecoverable error (seed lost, pickup timeout).
void goToHome() {
  digitalWrite(pumpPin, LOW);
  pumpActive = false;
  homeStepper(stepPin1, dirPin1, limitSwitch1, enablePin1, stepper1Homed, stepper1Pos);
  homeStepper(stepPin2, dirPin2, limitSwitch2, enablePin2, stepper2Homed, stepper2Pos);
  moveGantryTo(gantryRestAngle);
  moveGripperTo(gripperOpenAngle);
  Serial.println("ERROR: System homed after failure. Reset to restart.");
}

// ===== SEED PICKUP =====


// If no pickup, oscillates stepper 1 up and down 0.5 rounds repeatedly until pressure threshold is met or pickupMax ms elapses. Returns true on success.
bool tryPickupSeed() {
  unsigned long start = millis();

  while (millis() - start < pickupWait) {
    if (readPressure() >= pressurePickedThreshold) return true;
    delay(200);
  }

  while (millis() - start < pickupMax) {
    if (readPressure() >= pressurePickedThreshold) return true;
    moveStepper1(roundsToSteps(0.5), true);
    if (readPressure() >= pressurePickedThreshold) return true;
    moveStepper1(roundsToSteps(0.5), false);
  }

  return false;
}

// Calls tryPickupSeed() up to seedPickupRetries + 1 times.

void waitForSeedPickup() {
  for (int attempt = 0; attempt <= seedPickupRetries; attempt++) {
    if (tryPickupSeed()) return;
  }
  Serial.println("ERROR: Seed pickup failed after all retries.");
  goToHome();
  while (true);
}

// Reads current pressure and triggers emergency home if it has dropped below pressureDropThreshold
void checkPressureOrHome() {
  float p = readPressure();
  if (p < pressureDropThreshold) {
    Serial.print("ERROR: Pressure dropped to ");
    Serial.print(p);
    Serial.println(" kPa — seed lost.");
    goToHome();
    while (true);
  }
}

// ===== SETUP =====
void setup() {
  pinMode(dirPin1,      OUTPUT);
  pinMode(stepPin1,     OUTPUT);
  pinMode(enablePin1,   OUTPUT);
  pinMode(limitSwitch1, INPUT_PULLUP);

  pinMode(dirPin2,      OUTPUT);
  pinMode(stepPin2,     OUTPUT);
  pinMode(enablePin2,   OUTPUT);
  pinMode(limitSwitch2, INPUT_PULLUP);

  pinMode(limitSwitchGantry,  INPUT_PULLUP);
  pinMode(limitSwitchGripper, INPUT_PULLUP);
  pinMode(triggerPin,         INPUT_PULLUP);

  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin,    LOW);
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);

  Serial.begin(9600);
  while (!Serial);

  delay(500);
  gantryServo.attach(gantryServoPin);
  gripperServo.attach(gripperServoPin);
  cuttingServo.attach(cuttingServoPin);
  gantryServo.write(gantryHomeAngle);
  gripperServo.write(90);
  cuttingServo.write(cuttingStartAngle);
  delay(500);

  homeStepper(stepPin1, dirPin1, limitSwitch1, enablePin1, stepper1Homed, stepper1Pos);
  homeStepper(stepPin2, dirPin2, limitSwitch2, enablePin2, stepper2Homed, stepper2Pos);
  homeGantry();
  homeGripper();
  moveGripperTo(gripperOpenAngle);

  runClogTest();
  pressureSmoothed = analogRead(pressureSensor);

  Serial.println("Ready.");
}

void loop() {
  static bool done = false;
  if (done) return;

  // ===== PHASE 1: USED PLUG REMOVAL =====

  waitForPress("Press button to begin.");

  moveGripperTo(gripperGripAngle);
  delay(1000);

  moveGantryTo(gantryRestAngle);
  delay(500);

  moveStepper1(roundsToSteps(plug_stepper1_downRounds), false);

  moveStepper2(roundsToSteps(plug_stepper2_digRounds), false);
  delay(15000);

  performCut();

  returnStepper1();

  moveGantryTo(gantryDiscardAngle);
  delay(500);

  returnStepper2();
  delay(500);

  // ===== PHASE 2: NEW PLUG & SEED =====

  moveGantryTo(gantrySubstratePick);
  delay(2500);

  moveStepper1(roundsToSteps(stepper1_firstDownRounds), false);

  moveStepper2(roundsToSteps(stepper2_downRounds), false);
  delay(500);

  returnStepper1();

  moveGantryTo(gantryRestAngle);

  moveStepper1(roundsToSteps(stepper1_secondDownRounds), false);

  returnStepper2();
  delay(1000);

  digitalWrite(pumpPin, HIGH);
  pumpActive = true;
  delay(500);

  waitForSeedPickup();

  moveStepper1(roundsToSteps(stepper1_upRounds), true);
  checkPressureOrHome();
  delay(200);

  moveGantryTo(gantrySeedPick);
  checkPressureOrHome();
  delay(200);

  moveStepper1(roundsToSteps(stepper1_thirdDownRounds), false);
  checkPressureOrHome();
  delay(500);

  digitalWrite(pumpPin, LOW);
  pumpActive = false;
  delay(1500);

  returnStepper1();
  delay(500);

  moveGantryTo(gantryRestAngle);

  moveGripperTo(gripperOpenAngle);
  delay(500);

  done = true;
  Serial.println("Cycle complete. Reset to run again.");
}