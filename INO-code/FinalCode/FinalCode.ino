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
const float stepsPerRevolution = 400.0; // steps per revolution for steppers
const int moveDelay1 = 300;             // gantry movement speed
const int moveDelay2 = 300;             // substrate gripper movement speed
const int slowDelay  = 500;             // gantry and substrate homing speed
const int safeOffset = 80;              // offset from microswitches

// ===== USED PLUG CALIBRATION =====
const float plug_stepper1_downRounds = 10.2;                          // rounds down to pick used substrate
const float plug_stepper2_diglength  = 3.75;                           // [cm] dig length
const float plug_stepper2_digRounds  = plug_stepper2_diglength / 0.8; // substrate gripper dig depth for used substrate

// ===== NEW PLUG & SEED CALIBRATION =====
const float stepper1_firstDownRounds  = 10.0; // rounds down to pick unused substrate
const float stepper1_secondDownRounds = 10.0; // rounds down to place unused substrate
const float stepper1_upRounds         = 3.0;  // rounds up after picking seed
const float stepper1_thirdDownRounds  = 5.0;  // rounds down to drop-off seed
const float stepper2_downRounds       = 2.0;  // substrate gripper dig for unused substrate

// ===== CUTTING SERVO CALIBRATION =====
const int cuttingStartAngle = 30;  // resting angle for cutter
const int cuttingEndAngle   = 150; // end angle for cutter
const int cuttingPauseMs    = 500; // resting time for cutter

// ===== GANTRY SERVO CALIBRATION =====
const int gantryHomingSpeed   = 10;  // ms pause per degree during homing sweep
const int gantrySafeOffset    = 23;  // degrees to back off after switch triggers
const int gantryHomeAngle     = 23;  // startup write angle before homing
const int gantryRestAngle     = 0;   // rest position relative to zero
const int gantryDiscardAngle  = 160; // angle of dropping substrate
const int gantrySeedPick      = 120;  // seed drop-off position
const int gantrySubstratePick = 160; // angle for picking new substrate
const int gantryMinDelay      = 2;   // ms per degree at peak speed
const int gantryMaxDelay      = 10;  // ms per degree at start/end

// ===== GRIPPER SERVO CALIBRATION =====
const int gripperHomingSpeed = 15; // ms per degree for homing gripper
const int gripperSafeOffset  = 5;  // offset from trigger switch
const int gripperOpenAngle   = 80; // open gripper position
const int gripperGripAngle   = 25; // angle for gripping substrate plug

// ===== PRESSURE CALIBRATION =====
const float V_MIN                   = 0.1727; // pressure offset
const float V_RANGE                 = 4.5;    // operating range of sensor
const float P_MAX                   = 500.0;  // maximum pressure measurement
const float pressureClogLimit       = 40.0;   // threshold for clog test
const float pressurePickedThreshold = 50.0;   // threshold for pickup sensing
const float pressureDropThreshold   = 50.0;   // threshold for dropped seed
const float pressureAlpha           = 0.1;    // smoothing factor
const int   clogTestDuration        = 5000;   // duration of clog test

// ===== SEED PICKUP RETRY =====
const int          seedPickupRetries = 2;     // extra attempts after first failure
const unsigned long pickupWait       = 4000;  // ms to wait before oscillating
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
// On every upward step, checks the limit switch — if triggered unexpectedly,
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
// On every upward step, checks the limit switch

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

// Returns stepper 1 exactly as many steps upward as it currently is below home, then resets position tracker to zero
void returnStepper1() {
  if (stepper1Pos <= 0) return;
  moveStepper1(stepper1Pos, true);
  stepper1Pos = 0;
}

// Returns stepper 2 exactly as many steps upward as it currently is below home, then resets position tracker to zero.
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

// Blocks execution until the physical trigger button is pressed and released. with debounce

void waitForPress(const char* message) {
  Serial.println(message);
  while (digitalRead(triggerPin) == LOW);
  while (digitalRead(triggerPin) == HIGH);
  delay(50);
}

// ===== PRESSURE =====

// Converts ADC voltage to absolute pressure in kPa using sensor-specific offset and range constants.
float voltageToPressure(float voltage) {
  return ((voltage - V_MIN) / V_RANGE) * P_MAX;
}

// Reads ADC, applies exponential moving average smoothing, converts to voltage then to kPa and returns the result.
float readPressure() {
  int raw = analogRead(pressureSensor);
  pressureSmoothed = pressureAlpha * raw + (1.0 - pressureAlpha) * pressureSmoothed;
  float voltage = pressureSmoothed * (5.0 / 1023.0);
  return voltageToPressure(voltage);
}

// Runs pump for clogTestDuration ms and monitors pressure.
// If pressure exceeds pressureClogLimit the program halts: blockage .
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

// ===== GANTRY SERVO =====

// Drives gantry servo toward decreasing angle until limit switch triggers,
// then backs off gantrySafeOffset degrees.

void homeGantry() {
  int angle = gantryServo.read();

  while (digitalRead(limitSwitchGantry) == HIGH) {
    angle--;
    if (angle < 0) {
      angle = 0;
      gantryServo.write(0);
      Serial.println("WARNING: Gantry switch not found. Using angle 0 as reference.");
      delay(300);
      gantryZero = 0;
      moveGantryTo(gantryHomeAngle);
      return;
    }
    gantryServo.write(angle);
    delay(gantryHomingSpeed);
  }

  angle = clampAngle(angle + gantrySafeOffset);
  gantryServo.write(angle);
  delay(300);
  gantryZero = angle;

  moveGantryTo(gantryHomeAngle);
}

// Moves gantry servo to a position expressed as degrees from gantryZero. Uses a sinusoidal speed profile

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

// Drives gripper servo toward increasing angle until limit switch triggers, then backs off gripperSafeOffset degrees. 

void homeGripper() {
  int angle = gripperServo.read();

  while (digitalRead(limitSwitchGripper) == HIGH) {
    angle++;
    if (angle > 180) {
      angle = 180;
      gripperServo.write(angle);
      Serial.println("WARNING: Gripper switch not found.");
      break;
    }
    gripperServo.write(angle);
    delay(gripperHomingSpeed);
  }

  angle = clampAngle(angle - gripperSafeOffset);
  gripperServo.write(angle);
  delay(300);
  gripperZero = angle;
}

// Moves gripper to a position expressed as degrees subtracted from gripperZero

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

// Executes five full cutting strokes by sweeping between cuttingStartAngle and cuttingEndAngle
void performCut() {
  cuttingServo.write(cuttingStartAngle); delay(cuttingPauseMs);
  cuttingServo.write(cuttingEndAngle);   delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle); delay(cuttingPauseMs);
  cuttingServo.write(cuttingEndAngle);   delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle); delay(cuttingPauseMs);
  cuttingServo.write(cuttingEndAngle);   delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle); delay(cuttingPauseMs);
  cuttingServo.write(cuttingEndAngle);   delay(cuttingPauseMs);
  cuttingServo.write(cuttingStartAngle);
}

// ===== EMERGENCY HOME =====

// Called on any unrecoverable error. Stops pump, re-homes both steppers via limit switches
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

// Single pickup attempt. Waits pickupWait ms stationary for pressure to rise.

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

// Reads pressure and triggers emergency home if below pressureDropThreshold, indicating seed was lost during transport.
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

  delay(100);
  gantryServo.attach(gantryServoPin);
  gripperServo.attach(gripperServoPin);
  cuttingServo.attach(cuttingServoPin);
  gantryServo.write(gantryHomeAngle);
  gripperServo.write(90);
  cuttingServo.write(cuttingStartAngle);
  delay(100);

  homeStepper(stepPin1, dirPin1, limitSwitch1, enablePin1, stepper1Homed, stepper1Pos);
  homeStepper(stepPin2, dirPin2, limitSwitch2, enablePin2, stepper2Homed, stepper2Pos);
  homeGantry();
  homeGripper();
  moveGripperTo(gripperOpenAngle);

  runClogTest();
  pressureSmoothed = analogRead(pressureSensor);

  Serial.println("Ready.");
}

// ===== LOOP =====
void loop() {
  static bool done = false;
  if (done) return;

  // ===== PHASE 1: USED PLUG REMOVAL =====

  waitForPress("Press button to begin.");

  moveGripperTo(gripperGripAngle);
  delay(1000);

  moveGantryTo(gantryHomeAngle);
  delay(100);

  moveStepper1(roundsToSteps(plug_stepper1_downRounds), false);

  moveStepper2(roundsToSteps(plug_stepper2_digRounds), false);
  delay(1000);                                                    // for end effector to reach root cutter

  performCut();

  returnStepper1();

  moveGantryTo(gantryDiscardAngle);
  delay(100);

  moveStepper1(roundsToSteps(stepper1_firstDownRounds), false);

  returnStepper2();
  delay(100);

  // ===== PHASE 2: NEW PLUG & SEED =====

  moveGantryTo(gantrySubstratePick);
  delay(2500);

  moveStepper2(roundsToSteps(stepper2_downRounds), false);
  delay(100);

  returnStepper1();

  moveGantryTo(gantryHomeAngle);

  moveStepper1(roundsToSteps(stepper1_secondDownRounds), false);

  returnStepper2();
  delay(100);

  digitalWrite(pumpPin, HIGH);
  pumpActive = true;
  delay(100);

  waitForSeedPickup();

  moveStepper1(roundsToSteps(stepper1_upRounds), true);
  checkPressureOrHome();
  delay(100);

  moveGantryTo(gantrySeedPick);
  checkPressureOrHome();
  delay(100);

  moveStepper1(roundsToSteps(stepper1_thirdDownRounds), false);
  checkPressureOrHome();
  delay(100);

  digitalWrite(pumpPin, LOW);
  pumpActive = false;
  delay(1500);

  returnStepper1();
  delay(100);

  moveGantryTo(gantryHomeAngle);

  moveGripperTo(gripperOpenAngle);
  delay(100);

  done = true;
  Serial.println("Cycle complete. Reset to run again.");
}