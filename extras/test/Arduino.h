// Minimal Arduino.h stub for host-side (off-target) testing.
//
// PIDEasy only needs millis() from the Arduino core, so this stub provides
// it backed by a fake clock the tests can drive directly. Putting this
// directory on the include path ahead of any real core makes src/PIDEasy.cpp
// compile with a desktop compiler.
//
// This file is NOT part of the library — extras/ is ignored by the Arduino
// build system.

#ifndef PIDEASY_TEST_ARDUINO_H
#define PIDEASY_TEST_ARDUINO_H

// Fake clock, defined in the test translation unit.
extern unsigned long fake_clock_ms;

inline unsigned long millis() { return fake_clock_ms; }

// Test helpers.
inline void setMillis(unsigned long ms) { fake_clock_ms = ms; }
inline void advanceMillis(unsigned long ms) { fake_clock_ms += ms; }

#endif
