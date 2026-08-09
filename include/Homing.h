#pragma once

// ============================================================
//  Homing.h  –  Limit-switch homing for both axes
//
//  Sequence
//  --------
//  1. Coarse: drive both axes simultaneously at full speed
//     until both limit switches trigger.
//  2. Fine  : for each axis independently –
//     a. Retract (away from switch) until switch releases.
//     b. Back off HOME_RETRACT_STEPS extra to clear the switch.
//     c. Re-probe slowly to find the exact trigger point.
//     Steps (a)-(c) repeat HOME_PASSES times.
//
//  After homing, posX and posY are reset to 0.
//
//  checkDrift()
//  ------------
//  Called after every completed move.  If the switch state
//  does not match the expected position (0 vs non-zero),
//  a single fine-home pass is run on that axis.
// ============================================================

#include <Arduino.h>
#include "Stepper.h"

class Homing {
public:
    // @param motorX   Reference to the X-axis Stepper
    // @param motorY   Reference to the Y-axis Stepper
    // @param pinXLim  Limit-switch GPIO for X (active LOW)
    // @param pinYLim  Limit-switch GPIO for Y (active LOW)
    // @param posX     Reference to the global X position (steps)
    // @param posY     Reference to the global Y position (steps)
    Homing(Stepper& motorX, Stepper& motorY,
           uint8_t pinXLim, uint8_t pinYLim,
           long& posX, long& posY);

    // Run the full coarse + fine homing sequence for both axes.
    void run();

    // Verify switch states against stored positions;
    // re-home any axis that appears to have drifted.
    void checkDrift();

private:
    Stepper& _mX;
    Stepper& _mY;
    uint8_t  _pinX;
    uint8_t  _pinY;
    long&    _posX;
    long&    _posY;

    // Fine-home a single axis.
    // @param motor     Stepper to drive
    // @param switchPin Limit-switch GPIO
    // @param passes    Number of retract + re-probe cycles
    void _refineAxis(Stepper& motor, uint8_t switchPin, int passes);
};
