#pragma once

#include "Defines.h"

inline bool IsPinPairStrapped(uint8_t pinA, uint8_t pinB)
{
    pinMode(pinA, OUTPUT);
    digitalWrite(pinA, LOW);
    pinMode(pinB, INPUT_PULLUP);
    delay(2);
    const bool forwardSense = (digitalRead(pinB) == LOW);

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, OUTPUT);
    digitalWrite(pinB, LOW);
    delay(2);
    const bool reverseSense = (digitalRead(pinA) == LOW);

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    return forwardSense && reverseSense;
}

// For continuous monitoring in a control loop, hold one pin low and read the other through its pull-up
// instead of rerunning the full bidirectional probe every cycle.
inline void BeginPinPairStrapMonitor(uint8_t drivePin, uint8_t sensePin)
{
    pinMode(drivePin, OUTPUT);
    digitalWrite(drivePin, LOW);
    pinMode(sensePin, INPUT_PULLUP);
}

inline bool IsPinPairStrapMonitorClosed(uint8_t sensePin)
{
    return digitalRead(sensePin) == LOW;
}

inline void EndPinPairStrapMonitor(uint8_t drivePin, uint8_t sensePin)
{
    pinMode(drivePin, INPUT_PULLUP);
    pinMode(sensePin, INPUT_PULLUP);
}
