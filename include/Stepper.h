#pragma once

// ============================================================
//  Stepper.h  –  Low-level 4-wire unipolar stepper driver
//
//  Encapsulates pin assignment, full-step sequencing,
//  and coil enable/disable for a single stepper motor.
//
//  Direction convention (matches original firmware):
//    step(+1)  →  toward home (negative position direction)
//    step(-1)  →  away from home (positive position direction)
// ============================================================

#include <Arduino.h>

class Stepper {
public:
    // --------------------------------------------------------
    //  Constructor
    //  @param pins  Array of 4 GPIO pin numbers [IN1..IN4]
    // --------------------------------------------------------
    Stepper(const uint8_t pins[4]);

    // Initialise GPIOs as OUTPUT (call from setup())
    void begin();

    // Advance the motor by one full step.
    // dir = +1  →  positive phase increment (toward home)
    // dir = -1  →  negative phase increment (away from home)
    void step(int dir);

    // Restore the current phase to GPIO (call after enable())
    void restorePhase();

    // De-energise all coils (saves power, reduces heat)
    void disable();

    // Re-energise coils at the last known phase
    void enable();

    // Returns true if any coil pin is currently HIGH
    bool isAnyCoilOn() const;

    // Returns true if coils are currently energised
    bool isEnabled() const { return _enabled; }

    // Current step-table index (0-3)
    int  phaseIndex() const { return _index; }

private:
    uint8_t _pins[4];
    int     _index   = 0;
    bool    _enabled = true;

    // Full-step sequence (4 states × 4 coils)
    static const uint8_t STEP_TABLE[4][4];

    void _output();
};
