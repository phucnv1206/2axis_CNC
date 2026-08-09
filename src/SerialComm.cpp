#include "SerialComm.h"

// ------------------------------------------------------------
SerialComm::SerialComm(Motion& motion, Homing& homing,
                       long& posX, long& posY,
                       uint32_t& lastMoveMs)
    : _motion(motion), _homing(homing),
      _posX(posX), _posY(posY),
      _lastMoveMs(lastMoveMs)
{}

// ------------------------------------------------------------
void SerialComm::begin() {
    _servo.attach(PIN_SERVO);
    penUp();
}

// ------------------------------------------------------------
void SerialComm::poll(bool machineIdle) {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (_buf.length() > 0) {
                if (machineIdle) {
                    _dispatch(_buf);
                } else {
                    // While moving only accept emergency stop
                    String tmp = _buf;
                    tmp.trim();
                    tmp.toUpperCase();
                    if (tmp == "S") {
                        _motion.emergencyStop();
                        printPos();
                    }
                }
                _buf = "";
            }
        } else {
            _buf += c;
        }
    }
}

// ------------------------------------------------------------
void SerialComm::printPos() const {
    Serial.print(F("POS:"));
    Serial.print(_posX / STEPS_PER_MM, 2);
    Serial.print(F(","));
    Serial.println(_posY / STEPS_PER_MM, 2);
}

// ------------------------------------------------------------
void SerialComm::penUp() {
    _servo.write(PEN_UP_ANGLE);
    Serial.println(F("Pen up."));
    printPos();
}

// ------------------------------------------------------------
void SerialComm::penDown() {
    _servo.write(PEN_DOWN_ANGLE);
    Serial.println(F("Pen down."));
    printPos();
}

// ------------------------------------------------------------
void SerialComm::_dispatch(String line) {
    line.trim();
    line.toUpperCase();

    // ---- Single-character / keyword commands ----
    if (line == "D") { penDown(); return; }
    if (line == "U") { penUp();   return; }
    if (line == "?") { printPos(); return; }

    if (line == "H") {
        Serial.println(F("Homing..."));
        _homing.run();
        _lastMoveMs = millis();
        printPos();
        return;
    }

    if (line == "LIMIT?") {
        Serial.print(F("LIMIT:"));
        Serial.println(AXIS_LIMIT_MM);
        printPos();
        return;
    }

    // ---- Move command: X<f> Y<f> [F<n>] ----
    int xIdx = line.indexOf('X');
    int yIdx = line.indexOf('Y');
    int fIdx = line.indexOf('F');

    if (xIdx == -1 || yIdx == -1 || yIdx < xIdx) {
        Serial.println(F("Syntax error. Examples:  X10 Y20  |  X10 Y10 F0  |  D  |  U"));
        printPos();
        return;
    }

    // Y value ends at 'F' (if present after Y) or end of string
    int yEnd = line.length();
    if (fIdx > yIdx) yEnd = fIdx;

    float xmm = line.substring(xIdx + 1, yIdx).toFloat();
    float ymm  = line.substring(yIdx + 1, yEnd).toFloat();

    // Ramp mode
    bool rampIn = true, rampOut = true;
    if (fIdx != -1) {
        int mode = line.substring(fIdx + 1).toInt();
        if (mode < 0 || mode > 3) mode = 3;
        rampIn  = (mode == 1 || mode == 3);
        rampOut = (mode == 2 || mode == 3);
    }

    // Bounds check
    if (xmm < 0 || xmm > AXIS_LIMIT_MM || ymm < 0 || ymm > AXIS_LIMIT_MM) {
        Serial.print(F("Error: position out of range 0–"));
        Serial.print(AXIS_LIMIT_MM);
        Serial.println(F(" mm."));
        printPos();
        return;
    }

    long tx = Motion::mmToSteps(xmm);
    long ty = Motion::mmToSteps(ymm);
    _motion.startMove(tx, ty, true, rampIn, rampOut);
    _lastMoveMs = millis();
}
