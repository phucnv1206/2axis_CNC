#include <ESP32Servo.h>

Servo sv;

#define X_axis 26
#define Y_axis 25

// 2048 step/rotation
// 0.0194565 mm/step => steps/mm = 51.4
#define STEPS_PER_MM 51.4f
#define AXIS_LIMIT_MM 120
#define MAX_STEPS ((long)(AXIS_LIMIT_MM * STEPS_PER_MM))  // ~5140 steps

//================= Pen raise/lower Servo =================
#define SERVO_PIN 21
#define PEN_UP_ANGLE 75     // pen raised, not touching paper
#define PEN_DOWN_ANGLE 96  // pen lowered, touching paper

//================= Motor 1 (X axis) =================
const uint8_t motor1Pins[4] = { 15, 2, 4, 16 };

//================= Motor 2 (Y axis) =================
const uint8_t motor2Pins[4] = { 17, 5, 18, 19 };

// Full-step sequence (4 states)
const uint8_t stepTable[4][4] = {
  { 1, 1, 0, 0 },
  { 0, 1, 1, 0 },
  { 0, 0, 1, 1 },
  { 1, 0, 0, 1 }
};

int stepIndex1 = 0;
int stepIndex2 = 0;

void outputMotor(const uint8_t pins[], int index) {
  for (int i = 0; i < 4; i++)
    digitalWrite(pins[i], stepTable[index][i]);
}

// dir = -1 : run in positive direction (increase position, move away from home)
// dir =  1 : run in negative direction (decrease position, move towards home switch)
void stepMotorX(int dir) {
  stepIndex1 += dir;
  if (stepIndex1 >= 4) stepIndex1 = 0;
  if (stepIndex1 < 0) stepIndex1 = 3;
  outputMotor(motor1Pins, stepIndex1);
}

void stepMotorY(int dir) {
  stepIndex2 += dir;
  if (stepIndex2 >= 4) stepIndex2 = 0;
  if (stepIndex2 < 0) stepIndex2 = 3;
  outputMotor(motor2Pins, stepIndex2);
}

#define STEP_PERIOD_MIN_US 2500    // us/step, fastest speed (~300Hz) - travel & coarse homing
#define STEP_PERIOD_MEDIUM_US 3333  // us/step, medium speed (~111Hz) - backing off switch
#define STEP_PERIOD_MAX_US 5000    // us/step, slowest speed (80Hz) - precise re-homing
#define RAMP_STEPS 60              // number of steps for acceleration/deceleration on straight path (F)
#define HOME_RETRACT_STEPS 60      // minimum retract steps away from switch to fully release button
#define HOME_PASSES 1               // number of retract + re-probe passes to determine precise home position

#define IDLE_DISABLE_MS 500UL  // ms of inactivity -> disable motor phases to save power/prevent heating

uint32_t lastTime = 0;
uint32_t lastMoveTime = 0;
bool motorsEnabled = true;

//======================= Current position (in steps, 0 = home) =======================
long posX = 0;
long posY = 0;

//======================= Linear interpolation state (Bresenham) =======================
struct MotionState {
  bool active = false;
  bool xIsMajor = true;  // axis with larger step count -> step every tick
  long stepsMajor = 0;   // total steps for major axis (ticks needed to complete)
  long stepsMinor = 0;   // total steps for minor axis
  long counted = 0;      // ticks executed
  long error = 0;        // Bresenham error accumulator
  int dirX = 1;
  int dirY = 1;
  bool rampIn = true;   // flag to accelerate at start of command
  bool rampOut = true;  // flag to decelerate at end of command
} motion;

long targetX = 0;
long targetY = 0;

//======================= Flag indicating home drift check is needed after full stop =======================
bool needDriftCheck = false;

//======================= Prototypes =======================
void startMove(long tx, long ty, bool verbose = true, bool rampIn = true, bool rampOut = true);
long mmToSteps(float mm);
uint32_t getCurrentPeriod();
void refineAxis(void (*stepFunc)(int), int switchPin, int passes);
void checkAxisDrift();
void penUp();
void penDown();
void printPos();
void disableMotors();
void enableMotors();

//======================= Report current position to host =======================
void printPos() {
  Serial.print(F("POS:"));
  Serial.print(posX / STEPS_PER_MM, 2);
  Serial.print(F(","));
  Serial.println(posY / STEPS_PER_MM, 2);
}

//======================= Disable/enable motor drive phases when idle =======================
void disableMotors() {
  if (!motorsEnabled) return;
  for (int i = 0; i < 4; i++) {
    digitalWrite(motor1Pins[i], LOW);
    digitalWrite(motor2Pins[i], LOW);
  }
  motorsEnabled = false;
  Serial.println(F("Idle: motor phases disabled."));
}

void enableMotors() {
  if (motorsEnabled) return;
  outputMotor(motor1Pins, stepIndex1);  // restore correct current phase first to avoid step loss
  outputMotor(motor2Pins, stepIndex2);
  motorsEnabled = true;
}

//======================= Pen raise/lower Servo =======================
void penUp() {
  sv.write(PEN_UP_ANGLE);
  Serial.println(F("Pen up."));
  printPos();
}

void penDown() {
  sv.write(PEN_DOWN_ANGLE);
  Serial.println(F("Pen down."));
  printPos();
}

//======================= Sequential fine homing per axis =======================
void refineAxis(void (*stepFunc)(int), int switchPin, int passes) {
  for (int pass = 0; pass < passes; pass++) {
    // 1. Retract from switch until signal is fully released (HIGH)
    int safetyCounter = 0;
    while (digitalRead(switchPin) == LOW && safetyCounter < 1000) {
      stepFunc(-1);  // run in positive direction (away from switch)
      delayMicroseconds(STEP_PERIOD_MEDIUM_US);
      safetyCounter++;
    }

    // Retract a fixed additional distance (HOME_RETRACT_STEPS) to ensure button is fully released
    for (int i = 0; i < HOME_RETRACT_STEPS; i++) {
      stepFunc(-1);
      delayMicroseconds(STEP_PERIOD_MEDIUM_US);
    }

    // 2. Probe back very slowly (negative direction) to find exact contact point
    while (digitalRead(switchPin) == HIGH) {
      stepFunc(1);  // run in negative direction (towards switch)
      delayMicroseconds(STEP_PERIOD_MAX_US);
    }
  }
}

void doHoming() {
  enableMotors();

  // Coarse homing: run BOTH AXES SIMULTANEOUSLY at fastest speed until limit switch is triggered
  Serial.println(F("Coarse homing both axes..."));
  while (digitalRead(X_axis) == HIGH || digitalRead(Y_axis) == HIGH) {
    if (digitalRead(X_axis) == HIGH) stepMotorX(1);  // run in negative direction
    if (digitalRead(Y_axis) == HIGH) stepMotorY(1);  // run in negative direction
    delayMicroseconds(STEP_PERIOD_MIN_US);
  }

  // Fine homing independently per axis
  Serial.println(F("Fine homing X axis..."));
  refineAxis(stepMotorX, X_axis, HOME_PASSES);
  posX = 0;

  Serial.println(F("Fine homing Y axis..."));
  refineAxis(stepMotorY, Y_axis, HOME_PASSES);
  posY = 0;

  delay(200);
}

//======================= Check home position drift after full stop =======================
void checkAxisDrift() {
  bool xSwitchOn = (digitalRead(X_axis) == LOW);
  bool ySwitchOn = (digitalRead(Y_axis) == LOW);

  bool xOk = (xSwitchOn == (posX == 0));
  bool yOk = (ySwitchOn == (posY == 0));

  if (!xOk) {
    Serial.println(F("Warning: X axis position mismatch with switch, re-homing X axis..."));
    refineAxis(stepMotorX, X_axis, 1);
    posX = 0;
  }
  if (!yOk) {
    Serial.println(F("Warning: Y axis position mismatch with switch, re-homing Y axis..."));
    refineAxis(stepMotorY, Y_axis, 1);
    posY = 0;
  }
}

//======================= Motor step & safety check for sudden limit switch triggers =======================
void doStepX() {
  stepMotorX(motion.dirX);
  posX += -motion.dirX;  // dir=-1 (positive dir) -> posX+1 ; dir=1 (negative dir) -> posX-1
  lastMoveTime = millis();

  // Only reset position to 0 if actively moving in negative direction (dirX == 1) and actually hitting switch
  // Avoids false triggers caused by mechanical vibration when starting from position 0
  if (motion.dirX == 1 && posX < 10 && digitalRead(X_axis) == LOW) {
    posX = 0;
  }
}

void doStepY() {
  stepMotorY(motion.dirY);
  posY += -motion.dirY;
  lastMoveTime = millis();

  if (motion.dirY == 1 && posY < 10 && digitalRead(Y_axis) == LOW) {
    posY = 0;
  }
}

long mmToSteps(float mm) {
  long s = (long)round(mm * STEPS_PER_MM);
  return constrain(s, 0L, (long)MAX_STEPS);
}

//======================= Initialize straight linear motion to (tx, ty) =======================
void startMove(long tx, long ty, bool verbose, bool rampIn, bool rampOut) {
  enableMotors();

  tx = constrain(tx, 0L, (long)MAX_STEPS);
  ty = constrain(ty, 0L, (long)MAX_STEPS);

  long dX = tx - posX;
  long dY = ty - posY;

  motion.dirX = (dX >= 0) ? -1 : 1;
  motion.dirY = (dY >= 0) ? -1 : 1;

  long stepsX = abs(dX);
  long stepsY = abs(dY);

  targetX = tx;
  targetY = ty;

  if (stepsX >= stepsY) {
    motion.xIsMajor = true;
    motion.stepsMajor = stepsX;
    motion.stepsMinor = stepsY;
  } else {
    motion.xIsMajor = false;
    motion.stepsMajor = stepsY;
    motion.stepsMinor = stepsX;
  }

  motion.error = motion.stepsMajor / 2;
  motion.counted = 0;
  motion.active = (motion.stepsMajor > 0);
  motion.rampIn = rampIn;
  motion.rampOut = rampOut;

  if (!verbose) return;

  if (!motion.active) {
    Serial.println(F("Already at target position, no movement needed."));
    printPos();
  } else {
    Serial.print(F("Starting move to X="));
    Serial.print(tx / STEPS_PER_MM, 2);
    Serial.print(F("mm Y="));
    Serial.print(ty / STEPS_PER_MM, 2);
    Serial.println(F("mm"));
  }
}

//======================= Execute 1 step of Bresenham interpolation =======================
void motionTick() {
  if (!motion.active) return;

  if (motion.xIsMajor) {
    doStepX();
    motion.error -= motion.stepsMinor;
    if (motion.error < 0) {
      doStepY();
      motion.error += motion.stepsMajor;
    }
  } else {
    doStepY();
    motion.error -= motion.stepsMinor;
    if (motion.error < 0) {
      doStepX();
      motion.error += motion.stepsMajor;
    }
  }

  motion.counted++;
  if (motion.counted >= motion.stepsMajor) {
    motion.active = false;
    posX = targetX;
    posY = targetY;
    needDriftCheck = true;
  }
}

//======================= Calculate current tick period (Soft start / stop ramp) =======================
uint32_t getCurrentPeriod() {
  if (!motion.active) return STEP_PERIOD_MIN_US;

  long i = motion.counted;
  long total = motion.stepsMajor;
  long rampLen = min((long)RAMP_STEPS, total / 2);
  if (rampLen <= 0) return STEP_PERIOD_MIN_US;

  long fromStart = motion.rampIn ? i : rampLen;
  long fromEnd = motion.rampOut ? (total - 1 - i) : rampLen;

  long pos = min(fromStart, fromEnd);
  if (pos >= rampLen) return STEP_PERIOD_MIN_US;
  return map(pos, 0, rampLen, STEP_PERIOD_MAX_US, STEP_PERIOD_MIN_US);
}

//======================= Parse commands from Serial Monitor (Circular arc R structure removed) =======================
void parseAndMove(String line) {
  line.trim();
  line.toUpperCase();

  if (line == "D") {
    penDown();
    return;
  }
  if (line == "U") {
    penUp();
    return;
  }
  if (line == "?") {
    printPos();
    return;
  }
  if (line == "H") {
    Serial.println(F("Homing..."));
    doHoming();
    lastMoveTime = millis();
    printPos();
    return;
  }
  if (line == "LIMIT?") {
    Serial.print(F("LIMIT:"));
    Serial.println(AXIS_LIMIT_MM);
    printPos();
    return;
  }

  int xIndex = line.indexOf('X');
  int yIndex = line.indexOf('Y');
  int fIndex = line.indexOf('F');

  if (xIndex == -1 || yIndex == -1 || yIndex < xIndex) {
    Serial.println(F("Invalid syntax. Example: X10 Y20 / X10 Y10 F0 / D / U"));
    printPos();
    return;
  }

  int yEnd = line.length();
  if (fIndex > yIndex && fIndex < yEnd) yEnd = fIndex;

  float xmm = line.substring(xIndex + 1, yIndex).toFloat();
  float ymm = line.substring(yIndex + 1, yEnd).toFloat();

  bool rampIn = true, rampOut = true;
  if (fIndex != -1) {
    int mode = line.substring(fIndex + 1).toInt();
    if (mode < 0 || mode > 3) mode = 3;
    rampIn = (mode == 1 || mode == 3);
    rampOut = (mode == 2 || mode == 3);
  }

  if (xmm < 0 || xmm > AXIS_LIMIT_MM || ymm < 0 || ymm > AXIS_LIMIT_MM) {
    Serial.print(F("Position out of bounds 0-"));
    Serial.print(AXIS_LIMIT_MM);
    Serial.println(F("mm."));
    printPos();
    return;
  }

  long tx = mmToSteps(xmm);
  long ty = mmToSteps(ymm);

  startMove(tx, ty, true, rampIn, rampOut);
}

void handleSerialInput() {
  static String buf = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        parseAndMove(buf);
        buf = "";
      }
    } else {
      buf += c;
    }
  }
}

bool isAnyPhaseOn() {
  for (int i = 0; i < 4; i++) {
    if (digitalRead(motor1Pins[i]) == HIGH) return true;
    if (digitalRead(motor2Pins[i]) == HIGH) return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(motor1Pins[i], OUTPUT);
    pinMode(motor2Pins[i], OUTPUT);
  }

  pinMode(X_axis, INPUT_PULLUP);
  pinMode(Y_axis, INPUT_PULLUP);

  sv.attach(SERVO_PIN);
  penUp();

  doHoming();

  Serial.println(F("Homing completed."));
  lastMoveTime = millis();
  printPos();
}

void loop() {
  uint32_t period = getCurrentPeriod();
  if (micros() - lastTime >= period) {
    lastTime += period;
    motionTick();
  }

  if (!motion.active) {
    if (needDriftCheck) {
      needDriftCheck = false;
      checkAxisDrift();
      lastMoveTime = millis();
      printPos();
    }
    handleSerialInput();

    if (motorsEnabled && (millis() - lastMoveTime > IDLE_DISABLE_MS)) {
      disableMotors();
      if (isAnyPhaseOn()) {
        for (int i = 0; i < 4; i++) {
          digitalWrite(motor1Pins[i], LOW);
          digitalWrite(motor2Pins[i], LOW);
        }
        Serial.println(F("Warning: Force disabled phases manually second time!"));
      }
    }

  } else {
    // Moving: listen for emergency stop 'S'
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'S' || c == 's') {
        motion.active = false;
        needDriftCheck = false;
        Serial.println(F("EMERGENCY STOP EXECUTED."));
        printPos();
      }
    }
  }
}