#include "Stepper.h"

// Full-step sequence: each row is one phase state [IN1, IN2, IN3, IN4]
const uint8_t Stepper::STEP_TABLE[4][4] = {
    { 1, 1, 0, 0 },
    { 0, 1, 1, 0 },
    { 0, 0, 1, 1 },
    { 1, 0, 0, 1 }
};

// ------------------------------------------------------------
Stepper::Stepper(const uint8_t pins[4]) {
    for (int i = 0; i < 4; i++) _pins[i] = pins[i];
}

// ------------------------------------------------------------
void Stepper::begin() {
    for (int i = 0; i < 4; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], LOW);
    }
}

// ------------------------------------------------------------
void Stepper::step(int dir) {
    _index += dir;
    if (_index >= 4) _index = 0;
    if (_index <  0) _index = 3;
    _output();
}

// ------------------------------------------------------------
void Stepper::restorePhase() {
    _output();
    _enabled = true;
}

// ------------------------------------------------------------
void Stepper::disable() {
    for (int i = 0; i < 4; i++) digitalWrite(_pins[i], LOW);
    _enabled = false;
}

// ------------------------------------------------------------
void Stepper::enable() {
    restorePhase();
}

// ------------------------------------------------------------
bool Stepper::isAnyCoilOn() const {
    for (int i = 0; i < 4; i++)
        if (digitalRead(_pins[i]) == HIGH) return true;
    return false;
}

// ------------------------------------------------------------
void Stepper::_output() {
    for (int i = 0; i < 4; i++)
        digitalWrite(_pins[i], STEP_TABLE[_index][i]);
}
