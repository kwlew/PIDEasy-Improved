// Minimal Arduino.h stub for host-side (off-target) testing.
//
// PIDEasy only needs micros() from the Arduino core, so this stub provides
// it backed by a fake clock the tests can drive directly. Putting this
// directory on the include path ahead of any real core makes src/PIDEasy.cpp
// compile with a desktop compiler.
//
// The clock is 32 bits wide, like micros() on every Arduino core, so a
// rollover test behaves as on the board even where unsigned long is 64 bits.
//
// This file is NOT part of the library — extras/ is ignored by the Arduino
// build system.

#ifndef PIDEASY_TEST_ARDUINO_H
#define PIDEASY_TEST_ARDUINO_H

#include <stdint.h>

// Fake clock in microseconds, defined in the test translation unit.
extern uint32_t fake_clock_us;

inline unsigned long micros() { return fake_clock_us; }
inline unsigned long millis() { return fake_clock_us / 1000UL; }

// Test helpers.
inline void setMicros(uint32_t us) { fake_clock_us = us; }
inline void advanceMicros(uint32_t us) { fake_clock_us += us; }
inline void setMillis(unsigned long ms) { fake_clock_us = (uint32_t)(ms * 1000UL); }
inline void advanceMillis(unsigned long ms) { fake_clock_us += (uint32_t)(ms * 1000UL); }

#endif
