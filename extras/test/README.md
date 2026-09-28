# Host tests

Regression tests for the `PID` class that build and run on a **desktop compiler** —
no board, no upload, no serial monitor. Run these before a competition, and after
any change to `src/PIDEasy.cpp`.

Everything here lives under `extras/`, which the Arduino build system ignores
completely. None of it ships to the board or affects sketch size.

## Running them

Windows (PowerShell):

```powershell
.\extras\test\run_tests.ps1
```

Linux / macOS / Git Bash:

```bash
./extras/test/run_tests.sh
```

Or directly, from the repository root:

```bash
g++ -std=c++11 -Wall -Wextra -Iextras/test -Isrc extras/test/test_pideasy.cpp src/PIDEasy.cpp -o test_pideasy && ./test_pideasy
```

The runner exits non-zero if any check fails, so it drops straight into CI.

You need a C++ compiler on `PATH`. On Windows, `g++` from
[MSYS2](https://www.msys2.org/) works; set `$env:CXX` / `$CXX` to use a different one.

## How it works

`Arduino.h` here is a **stub**, not the real Arduino core. The library only needs
`micros()`, so the stub provides it backed by a fake 32-bit clock that the tests
drive directly:

```cpp
setMillis(1000);        // jump the clock to an absolute value (ms)
advanceMillis(50);      // move it forward (ms)
setMicros(1500);        // same, in microseconds
advanceMicros(250);
```

Passing `-Iextras/test` ahead of any real core makes `#include <Arduino.h>` in
`src/PIDEasy.h` resolve to the stub. That is the whole trick — because the fake
clock is controllable, timing behavior that is awkward to test on hardware
becomes trivial: a 5-second pause, a `micros()` rollover, or a loop running at
two different rates all happen instantly and deterministically.

The clock is 32 bits wide, like `micros()` on every Arduino core, so the rollover
test really wraps even on a 64-bit PC, where `unsigned long` is 64 bits.

`no_alias_check.cpp` is compiled (syntax only) with `PIDEASY_NO_PID_ALIAS` next to
a class of its own called `PID`, to prove the opt-out leaves that name free.

If the library ever needs another Arduino symbol, add it to the stub.

## What is covered

| Group | Checks |
|---|---|
| `setTunings` / gain getters | gains update; the switch is bumpless (a `ki` change keeps the output); new `ki` applies to future error only; `ki = 0` clears the I-term |
| Telemetry | `getP`/`getI`/`getD` match the terms; sum equals pre-clamp output; cleared by `reset()` |
| `setIntegralLimit` / windup | caps the I-term in output units; survives a `ki` retune; disable; argument swapping; windup still wins when tighter; no default windup cap; I-term bounded by the output range width; a one-sided range still lets I go negative |
| Conditional integration | holds the integral down while saturated; recovers sooner; never blocks integration that unwinds saturation |
| dt / resume | resume skips integral and derivative; normal cycles resume; cap disable; **`micros()` rollover**; `getDeltaTime`/`wasResumed`; a 120 ms loop still integrates; a 1.5 ms loop gives an unquantized derivative |
| Robustness | NaN / infinite error holds the output and does not consume elapsed time; a 4 kHz loop integrates real time; the first sample does not integrate |
| `update(setpoint, measurement)` | no derivative kick on a setpoint step; same D as `compute(error)` with a fixed setpoint; internal timer; `dt = 0` and NaN hold; `updateUs`/`computeUs` |
| `setContinuousInput` | 350 → 10 turns +20 both ways; `compute(error)` wraps; many turns wrap; derivative across ±180 is the short way; disable |
| `setTolerance` / `atSetpoint` | error band; settle time; leaving the band resets it; rate tolerance rejects a fast sweep; `reset()` clears it |
| Feedforward / min output / ramp | `kF`·sp + `kS`·sign; not applied to `compute()`; minimum output lifts small outputs but not inside tolerance; ramp limits both directions and soft-starts after `reset()` |
| Seconds / integral helpers | `computeSeconds`/`updateSeconds`; `dt <= 0` and NaN hold; `resetIntegral`; `setIntegral` with limits |
| Derivative filter | fixed coefficient is loop-rate dependent (the bug); time constant is not; matches the analytic step response; mode switching both ways |
| Backwards compatibility | deprecated seconds overload (including `dt = 0` → 1 s), `computeMs`, constrain, sign-change damping, the misspelled `setSmoothingDerivate` alias, argument swapping |
| API shape | getters callable through a `const` reference; `PID` aliases `PIDEasy` |

Two of these are worth calling out as the reason the suite exists:

- **`micros()` rollover** happens every ~71 minutes of uptime. You will rarely
  hit it during a run, and you cannot practically test it on hardware — but a
  regression there would corrupt dt.
- **Loop-rate independence of the derivative filter** is measured by running the
  same 200 ms of wall clock at 10 ms and 50 ms per sample and comparing. The
  fixed-coefficient path returns 87.8 vs 34.4; the time-constant path returns
  85.1 vs 80.2, both near the analytic 86.5.

## CI

`.github/workflows/ci.yml` runs this suite on every push (g++ with AddressSanitizer
and UBSan, and clang++, both with `-Werror`), runs `arduino-lint`, and compiles every
example for Arduino Uno, Mega, Raspberry Pi Pico and ESP32.

## Adding a test

Add a `check("name", condition)` inside the relevant `test_*()` function, or write
a new one and call it from `main()`. `check` counts failures and the process exit
code reflects them; there is no framework to learn and nothing to install.
