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
`millis()`, so the stub provides it backed by a fake clock that the tests drive
directly:

```cpp
setMillis(1000);      // jump the clock to an absolute value
advanceMillis(50);    // move it forward
```

Passing `-Iextras/test` ahead of any real core makes `#include <Arduino.h>` in
`src/PIDEasy.h` resolve to the stub. That is the whole trick — because the fake
clock is controllable, timing behavior that is awkward to test on hardware
becomes trivial: a 5-second pause, a `millis()` rollover, or a loop running at
two different rates all happen instantly and deterministically.

If the library ever needs another Arduino symbol, add it to the stub.

## What is covered

| Group | Checks |
|---|---|
| `setTunings` / gain getters | gains update; integral survives a gain change (bumpless mode switch) |
| Telemetry | `getP`/`getI`/`getD` match the terms; sum equals pre-clamp output; cleared by `reset()` |
| `setIntegralLimit` | caps the I-term in output units; survives a `ki` retune; disable; argument swapping; windup still wins when tighter |
| Conditional integration | holds the integral down while saturated; recovers sooner; never blocks integration that unwinds saturation |
| dt / resume | resume skips integral and derivative; normal cycles resume; cap disable; **`millis()` rollover** |
| Derivative filter | fixed coefficient is loop-rate dependent (the bug); time constant is not; matches the analytic step response; mode switching both ways |
| Backwards compatibility | seconds overload, `computeMs`, constrain, sign-change damping, the misspelled `setSmoothingDerivate` alias, argument swapping |

Two of these are worth calling out as the reason the suite exists:

- **`millis()` rollover** happens once every ~49 days of uptime. You will never
  hit it during a run, and you cannot practically test it on hardware — but a
  regression there would corrupt dt.
- **Loop-rate independence of the derivative filter** is measured by running the
  same 200 ms of wall clock at 10 ms and 50 ms per sample and comparing. The
  fixed-coefficient path returns 87.8 vs 34.4; the time-constant path returns
  85.1 vs 80.2, both near the analytic 86.5.

## Adding a test

Add a `check("name", condition)` inside the relevant `test_*()` function, or write
a new one and call it from `main()`. `check` counts failures and the process exit
code reflects them; there is no framework to learn and nothing to install.
