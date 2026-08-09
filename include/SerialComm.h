#pragma once

// ============================================================
//  SerialComm.h  –  Serial command parser & dispatcher
//
//  Supported commands (115200 baud, LF or CR terminated)
//  -------------------------------------------------------
//  D              Pen down
//  U              Pen up
//  ?              Report current position  → "POS:x.xx,y.yy"
//  H              Run full homing sequence
//  LIMIT?         Report axis limit        → "LIMIT:<mm>"
//  X<f> Y<f>      Move to absolute position in mm
//  X<f> Y<f> F<n> Move with ramp mode:
//                   F0 = none | F1 = accel only
//                   F2 = decel only | F3 = full (default)
//  S              Emergency stop (accepted while moving)
// ============================================================

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "Motion.h"
#include "Homing.h"

class SerialComm {
public:
    // @param motion     Reference to the Motion controller
    // @param homing     Reference to the Homing controller
    // @param posX       Reference to global X position (steps)
    // @param posY       Reference to global Y position (steps)
    // @param lastMoveMs Reference to the idle-timer timestamp
    SerialComm(Motion& motion, Homing& homing,
               long& posX, long& posY,
               uint32_t& lastMoveMs);

    // Attach the pen servo (call once in setup())
    void begin();

    // Read and process all pending bytes from Serial.
    // Only dispatches move commands when the machine is idle.
    void poll(bool machineIdle);

    // Print current position to Serial  →  "POS:x.xx,y.yy"
    void printPos() const;

    // Pen control (public so main.cpp can call them directly if needed)
    void penUp();
    void penDown();

private:
    Motion&   _motion;
    Homing&   _homing;
    long&     _posX;
    long&     _posY;
    uint32_t& _lastMoveMs;

    Servo    _servo;
    String   _buf;          // line accumulation buffer

    // Parse and dispatch a complete command line
    void _dispatch(String line);
};
