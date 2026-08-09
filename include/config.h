#pragma once

// ============================================================
//  config.h  –  Compile-time configuration for 2-axis CNC plotter
//
//  Edit the values in this file to adapt the firmware to your
//  hardware without touching any other source file.
// ============================================================

// ------------------------------------------------------------
//  Mechanical parameters
//  2048 steps/rev, lead-screw pitch => 0.0194565 mm/step
// ------------------------------------------------------------
#define STEPS_PER_MM      51.4f
#define AXIS_LIMIT_MM     120
#define MAX_STEPS         ((long)(AXIS_LIMIT_MM * STEPS_PER_MM))

// ------------------------------------------------------------
//  GPIO – limit switches (active LOW, INPUT_PULLUP)
// ------------------------------------------------------------
#define PIN_X_LIMIT       26
#define PIN_Y_LIMIT       25

// ------------------------------------------------------------
//  GPIO – stepper motors (full-step, 4-wire unipolar/bipolar)
// ------------------------------------------------------------
#define MOTOR_X_PIN1      15
#define MOTOR_X_PIN2       2
#define MOTOR_X_PIN3       4
#define MOTOR_X_PIN4      16

#define MOTOR_Y_PIN1      17
#define MOTOR_Y_PIN2       5
#define MOTOR_Y_PIN3      18
#define MOTOR_Y_PIN4      19

// ------------------------------------------------------------
//  GPIO – pen-lift servo
// ------------------------------------------------------------
#define PIN_SERVO         21
#define PEN_UP_ANGLE      75    // degrees – pen raised
#define PEN_DOWN_ANGLE    96    // degrees – pen on paper

// ------------------------------------------------------------
//  Stepper speed & ramping
// ------------------------------------------------------------
#define STEP_PERIOD_FAST_US    2500   // us/step – travel & coarse home (~400 Hz)
#define STEP_PERIOD_MED_US     3333   // us/step – retract from switch  (~300 Hz)
#define STEP_PERIOD_SLOW_US    5000   // us/step – fine probe            (200 Hz)
#define RAMP_STEPS             60     // accel/decel ramp length (steps)

// ------------------------------------------------------------
//  Homing
// ------------------------------------------------------------
#define HOME_RETRACT_STEPS    60   // extra steps after coarse hit to fully release switch
#define HOME_PASSES            1   // retract + re-probe cycles per axis for fine homing

// ------------------------------------------------------------
//  Idle coil-disable timeout
// ------------------------------------------------------------
#define IDLE_DISABLE_MS   500UL   // ms of inactivity → coils off

// ------------------------------------------------------------
//  Serial
// ------------------------------------------------------------
#define SERIAL_BAUD       115200

// ------------------------------------------------------------
//  MotionState  –  runtime state for one linear move
// ------------------------------------------------------------
struct MotionState {
    bool  active     = false;
    bool  xIsMajor   = true;  // axis with more steps drives the tick loop
    long  stepsMajor = 0;
    long  stepsMinor = 0;
    long  counted    = 0;     // ticks executed so far
    long  error      = 0;     // Bresenham error accumulator
    int   dirX       = 1;     // +1 toward home, -1 away from home
    int   dirY       = 1;
    bool  rampIn     = true;
    bool  rampOut    = true;
};
