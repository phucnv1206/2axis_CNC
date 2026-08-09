#include <Arduino.h>
#include "config.h"
#include "Stepper.h"
#include "Homing.h"
#include "Motion.h"
#include "SerialComm.h"

// ============================================================
//  Hardware objects
// ============================================================
static const uint8_t PINS_X[4] = { MOTOR_X_PIN1, MOTOR_X_PIN2, MOTOR_X_PIN3, MOTOR_X_PIN4 };
static const uint8_t PINS_Y[4] = { MOTOR_Y_PIN1, MOTOR_Y_PIN2, MOTOR_Y_PIN3, MOTOR_Y_PIN4 };

Stepper motorX(PINS_X);
Stepper motorY(PINS_Y);

// ============================================================
//  Shared state (position, timing)
// ============================================================
long     posX        = 0;
long     posY        = 0;
uint32_t lastMoveMs  = 0;
uint32_t lastTickUs  = 0;

// ============================================================
//  Subsystem controllers
// ============================================================
Motion     motion    (motorX, motorY, PIN_X_LIMIT, PIN_Y_LIMIT, posX, posY);
Homing     homing    (motorX, motorY, PIN_X_LIMIT, PIN_Y_LIMIT, posX, posY);
SerialComm serial    (motion, homing, posX, posY, lastMoveMs);

// ============================================================
//  setup
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);

    // Initialise stepper drivers
    motorX.begin();
    motorY.begin();

    // Limit switches – active LOW, internal pull-up
    pinMode(PIN_X_LIMIT, INPUT_PULLUP);
    pinMode(PIN_Y_LIMIT, INPUT_PULLUP);

    // Attach servo, lift pen
    serial.begin();

    // Home on power-up
    homing.run();

    lastMoveMs = millis();
    serial.printPos();
}

// ============================================================
//  loop
// ============================================================
void loop() {
    // ---- Time-driven step tick ----
    uint32_t period = motion.getCurrentPeriod();
    if (micros() - lastTickUs >= period) {
        lastTickUs += period;
        motion.tick();
    }

    // ---- Post-move drift check ----
    if (motion.needsDriftCheck()) {
        motion.clearDriftFlag();
        homing.checkDrift();
        lastMoveMs = millis();
        serial.printPos();
    }

    // ---- Serial input (idle: full commands | moving: 'S' only) ----
    serial.poll(!motion.isActive());

    // ---- Idle coil-disable ----
    if (!motion.isActive()) {
        bool xCoilsOn = motorX.isEnabled();
        bool yCoilsOn = motorY.isEnabled();

        if ((xCoilsOn || yCoilsOn) &&
            (millis() - lastMoveMs > IDLE_DISABLE_MS))
        {
            motorX.disable();
            motorY.disable();
            Serial.println(F("Idle: motor coils disabled."));

            // Safety: force-verify all pins are LOW
            if (motorX.isAnyCoilOn() || motorY.isAnyCoilOn()) {
                motorX.disable();
                motorY.disable();
                Serial.println(F("Warning: forced second coil-off pass!"));
            }
        }
    }
}
