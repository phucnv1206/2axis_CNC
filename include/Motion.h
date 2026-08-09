#pragma once

// ============================================================
//  Motion.h  –  Bresenham 2-axis linear interpolation
//               with trapezoidal speed ramp
//
//  Usage
//  -----
//  1. Call begin() once in setup().
//  2. Call startMove() to queue a move.
//  3. Call tick() every STEP_PERIOD_*_US microseconds from loop()
//     (driven by the caller's timer logic).
//  4. Poll isActive() to know when the move is done.
//  5. Call getCurrentPeriod() to get the correct inter-tick delay.
//
//  Ramp mode (F parameter in serial commands):
//    F0 – no ramp       (rampIn=false, rampOut=false)
//    F1 – accel only    (rampIn=true,  rampOut=false)
//    F2 – decel only    (rampIn=false, rampOut=true)
//    F3 – full ramp     (rampIn=true,  rampOut=true)  ← default
//
//  Position convention:
//    posX / posY are in steps; 0 = home (limit switch position).
//    Positive moves away from home.
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "Stepper.h"

class Motion {
public:
    // @param motorX    X-axis Stepper
    // @param motorY    Y-axis Stepper
    // @param pinXLim   X limit-switch GPIO
    // @param pinYLim   Y limit-switch GPIO
    // @param posX      Reference to global X position (steps)
    // @param posY      Reference to global Y position (steps)
    Motion(Stepper& motorX, Stepper& motorY,
           uint8_t pinXLim, uint8_t pinYLim,
           long& posX, long& posY);

    // Initialise (nothing hardware-specific here; kept for symmetry)
    void begin();

    // Queue a linear move to (targetX, targetY) in steps.
    // @param tx       Target X in steps
    // @param ty       Target Y in steps
    // @param verbose  Print move info to Serial
    // @param rampIn   Accelerate at start
    // @param rampOut  Decelerate at end
    void startMove(long tx, long ty,
                   bool verbose  = true,
                   bool rampIn   = true,
                   bool rampOut  = true);

    // Execute one Bresenham interpolation tick.
    // Call this from loop() when the timer fires.
    void tick();

    // Return the inter-tick period for the current ramp position.
    uint32_t getCurrentPeriod() const;

    // True while a move is in progress.
    bool isActive()        const { return _state.active; }

    // True if a drift check should be run (set after move completes).
    bool needsDriftCheck() const { return _driftPending; }
    void clearDriftFlag()        { _driftPending = false; }

    // Emergency stop – halts motion immediately.
    void emergencyStop();

    // Utility: convert mm to steps (clamped to [0, MAX_STEPS])
    static long mmToSteps(float mm);

private:
    Stepper& _mX;
    Stepper& _mY;
    uint8_t  _pinX;
    uint8_t  _pinY;
    long&    _posX;
    long&    _posY;

    MotionState _state;
    long        _targetX      = 0;
    long        _targetY      = 0;
    bool        _driftPending = false;

    // Single-step helpers that also enforce limit-switch safety
    void _doStepX();
    void _doStepY();
};
