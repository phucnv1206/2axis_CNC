#include "Motion.h"

// ------------------------------------------------------------
Motion::Motion(Stepper& motorX, Stepper& motorY,
               uint8_t pinXLim, uint8_t pinYLim,
               long& posX, long& posY)
    : _mX(motorX), _mY(motorY),
      _pinX(pinXLim), _pinY(pinYLim),
      _posX(posX), _posY(posY)
{}

// ------------------------------------------------------------
void Motion::begin() {
    // Nothing hardware-specific; MotionState initialised inline.
}

// ------------------------------------------------------------
void Motion::startMove(long tx, long ty,
                       bool verbose,
                       bool rampIn, bool rampOut)
{
    _mX.enable();
    _mY.enable();

    tx = constrain(tx, 0L, (long)MAX_STEPS);
    ty = constrain(ty, 0L, (long)MAX_STEPS);

    long dX = tx - _posX;
    long dY = ty - _posY;

    // dir = -1 → positive (away from home) | dir = +1 → negative (toward home)
    _state.dirX = (dX >= 0) ? -1 : 1;
    _state.dirY = (dY >= 0) ? -1 : 1;

    long stepsX = abs(dX);
    long stepsY = abs(dY);

    _targetX = tx;
    _targetY = ty;

    if (stepsX >= stepsY) {
        _state.xIsMajor   = true;
        _state.stepsMajor = stepsX;
        _state.stepsMinor = stepsY;
    } else {
        _state.xIsMajor   = false;
        _state.stepsMajor = stepsY;
        _state.stepsMinor = stepsX;
    }

    _state.error   = _state.stepsMajor / 2;
    _state.counted = 0;
    _state.active  = (_state.stepsMajor > 0);
    _state.rampIn  = rampIn;
    _state.rampOut = rampOut;

    if (!verbose) return;

    if (!_state.active) {
        Serial.println(F("Already at target, no move needed."));
    } else {
        Serial.print(F("Moving to X="));
        Serial.print(tx / STEPS_PER_MM, 2);
        Serial.print(F("mm  Y="));
        Serial.print(ty / STEPS_PER_MM, 2);
        Serial.println(F("mm"));
    }
}

// ------------------------------------------------------------
void Motion::tick() {
    if (!_state.active) return;

    // Bresenham: major axis always steps; minor axis steps when error < 0
    if (_state.xIsMajor) {
        _doStepX();
        _state.error -= _state.stepsMinor;
        if (_state.error < 0) {
            _doStepY();
            _state.error += _state.stepsMajor;
        }
    } else {
        _doStepY();
        _state.error -= _state.stepsMinor;
        if (_state.error < 0) {
            _doStepX();
            _state.error += _state.stepsMajor;
        }
    }

    _state.counted++;
    if (_state.counted >= _state.stepsMajor) {
        _state.active = false;
        _posX         = _targetX;
        _posY         = _targetY;
        _driftPending = true;
    }
}

// ------------------------------------------------------------
uint32_t Motion::getCurrentPeriod() const {
    if (!_state.active) return STEP_PERIOD_FAST_US;

    long i       = _state.counted;
    long total   = _state.stepsMajor;
    long rampLen = min((long)RAMP_STEPS, total / 2);
    if (rampLen <= 0) return STEP_PERIOD_FAST_US;

    long fromStart = _state.rampIn  ? i               : rampLen;
    long fromEnd   = _state.rampOut ? (total - 1 - i) : rampLen;

    long pos = min(fromStart, fromEnd);
    if (pos >= rampLen) return STEP_PERIOD_FAST_US;

    return (uint32_t)map(pos, 0, rampLen, STEP_PERIOD_SLOW_US, STEP_PERIOD_FAST_US);
}

// ------------------------------------------------------------
void Motion::emergencyStop() {
    _state.active = false;
    _driftPending = false;
    Serial.println(F("EMERGENCY STOP."));
}

// ------------------------------------------------------------
long Motion::mmToSteps(float mm) {
    long s = (long)round(mm * STEPS_PER_MM);
    return constrain(s, 0L, (long)MAX_STEPS);
}

// ------------------------------------------------------------
//  Private step helpers – update position and guard against
//  spurious limit-switch triggers near position 0
// ------------------------------------------------------------
void Motion::_doStepX() {
    _mX.step(_state.dirX);
    _posX += -_state.dirX;   // dir=-1 (pos move) → posX++ | dir=+1 → posX--

    // Snap to 0 only when moving toward home and switch triggers
    if (_state.dirX == 1 && _posX < 10 && digitalRead(_pinX) == LOW) {
        _posX = 0;
    }
}

void Motion::_doStepY() {
    _mY.step(_state.dirY);
    _posY += -_state.dirY;

    if (_state.dirY == 1 && _posY < 10 && digitalRead(_pinY) == LOW) {
        _posY = 0;
    }
}
