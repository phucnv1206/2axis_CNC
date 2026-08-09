#include "Homing.h"
#include "config.h"

// ------------------------------------------------------------
Homing::Homing(Stepper& motorX, Stepper& motorY,
               uint8_t pinXLim, uint8_t pinYLim,
               long& posX, long& posY)
    : _mX(motorX), _mY(motorY),
      _pinX(pinXLim), _pinY(pinYLim),
      _posX(posX), _posY(posY)
{}

// ------------------------------------------------------------
void Homing::run() {
    _mX.enable();
    _mY.enable();

    // ---- Coarse phase: both axes simultaneously ----
    Serial.println(F("Homing: coarse pass (both axes)..."));
    while (digitalRead(_pinX) == HIGH || digitalRead(_pinY) == HIGH) {
        if (digitalRead(_pinX) == HIGH) _mX.step(+1);   // toward home
        if (digitalRead(_pinY) == HIGH) _mY.step(+1);
        delayMicroseconds(STEP_PERIOD_FAST_US);
    }

    // ---- Fine phase: X axis ----
    Serial.println(F("Homing: fine pass X axis..."));
    _refineAxis(_mX, _pinX, HOME_PASSES);
    _posX = 0;

    // ---- Fine phase: Y axis ----
    Serial.println(F("Homing: fine pass Y axis..."));
    _refineAxis(_mY, _pinY, HOME_PASSES);
    _posY = 0;

    delay(200);
    Serial.println(F("Homing complete."));
}

// ------------------------------------------------------------
void Homing::checkDrift() {
    bool xTriggered = (digitalRead(_pinX) == LOW);
    bool yTriggered = (digitalRead(_pinY) == LOW);

    // Switch should be triggered if and only if position == 0
    if (xTriggered != (_posX == 0)) {
        Serial.println(F("Warning: X position/switch mismatch – re-homing X..."));
        _refineAxis(_mX, _pinX, 1);
        _posX = 0;
    }
    if (yTriggered != (_posY == 0)) {
        Serial.println(F("Warning: Y position/switch mismatch – re-homing Y..."));
        _refineAxis(_mY, _pinY, 1);
        _posY = 0;
    }
}

// ------------------------------------------------------------
void Homing::_refineAxis(Stepper& motor, uint8_t switchPin, int passes) {
    for (int pass = 0; pass < passes; pass++) {
        // 1. Retract until switch fully releases
        int safety = 0;
        while (digitalRead(switchPin) == LOW && safety < 1000) {
            motor.step(-1);   // away from switch (positive direction)
            delayMicroseconds(STEP_PERIOD_MED_US);
            safety++;
        }

        // 2. Extra retract to guarantee switch release
        for (int i = 0; i < HOME_RETRACT_STEPS; i++) {
            motor.step(-1);
            delayMicroseconds(STEP_PERIOD_MED_US);
        }

        // 3. Slow probe back to find exact trigger point
        while (digitalRead(switchPin) == HIGH) {
            motor.step(+1);   // toward switch (negative direction)
            delayMicroseconds(STEP_PERIOD_SLOW_US);
        }
    }
}
