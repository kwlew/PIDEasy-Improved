# PIDEasy-Improved — Library Reference

Reference for writing and auditing robot code that uses this library (RoboCup Junior, WRO, line followers, sumo, anything with motors and sensors).
Covers version 1.2.0. One class: `PIDEasy` (with `PID` as an alias), declared in `src/PIDEasy.h`, implemented in `src/PIDEasy.cpp`.

## ⚠️ Behavior changes in 1.2.0

Every 1.1.x sketch still compiles. These changes can alter how a tuned robot behaves:

1. **`compute(error)` times itself with `micros()` instead of `millis()`.** dt is now accurate at 1–2 kHz, where the old whole-millisecond timing made the derivative jitter by up to ±50%. The D-term will look calmer. Any gain you tuned to compensate for that jitter may need re-checking.
2. **The I-term is stored in output units.** Changing `ki` with `setTunings()` no longer makes the output jump (1.1.x scaled the whole accumulated integral by the new `ki`). `setTunings()` with `ki = 0` now clears the I-term.
3. **No raw windup clamp by default.** 1.1.x clamped the raw integral to ±255, so a small `ki` could never contribute more than `ki × 255` (12.75 with `ki = 0.05`). Now the I-term is bounded by ± the *width* of the output range (±510 for the default −255…255), and conditional integration keeps it in check while saturated. `setWindUP()` still works exactly as before when you call it.
4. **`setMaxDeltaTime()` defaults to 500 ms (was 100 ms).** At 100 ms, robots with slow sensor loops (e.g. several ToF reads per cycle) had every sample treated as a pause, so I and D never acted. Use `wasResumed()` to check.
5. **The two-argument `compute(error, dt)` is deprecated.** It still works exactly as before, but compiling with warnings on now points you to `computeMs()` / `computeSeconds()`.

And from 1.1.1:

- A **NaN or infinite error** returns the previous output and changes nothing. It used to poison the integral until `reset()`.
- The **first** `compute(error)` call no longer integrates a phantom 1 ms. A call with no elapsed time returns the previous output unchanged. Before, it counted as 1 ms, which made the I-term grow several times too fast above 1 kHz.

### From 1.0.x (changed in 1.1.0)

- **Conditional integration is ON by default.** `setConditionalIntegration(false)` restores the old behavior.
- **Gaps longer than `setMaxDeltaTime()` are a *resume*.** For that one sample the integral and derivative are skipped, instead of taking a clamped step.

## Quick model of how it works

Each `compute*` / `update*` call does, in order:

0. **Hold checks.** If an input is NaN or infinite, or no time has passed (dt = 0 in the new variants, or the internal timer has not ticked), the call returns the previous output and changes nothing.
1. **Error.** `error` is either passed in (`compute*`) or `setpoint − measurement` (`update*`). With `setContinuousInput()` it is wrapped into ± half the range.
2. **Integral.** `I += ki × error × dt`, then I is clamped to ± the output range width, to `ki × setWindUP()` limits if set, and to `setIntegralLimit()` if set. **Skipped on a resume sample.**
3. **Damping.** If `error` changed sign versus the previous call (strictly), `I *= dampingFactor`.
4. **Derivative.** `compute*` differentiates the **error**; `update*` differentiates **−measurement**, so setpoint changes cause no kick. The difference is wrapped too with continuous input. The derivative is forced to 0 on the first sample, on a resume, and on the first sample after switching between `compute*` and `update*`. It is then low-pass filtered: `d = d + smoothing × (previous_d − d)`, with `smoothing` from `setSmoothingDerivative()` or `tau / (tau + dt)`.
5. **Sum.** `output = kp×error + I + kd×derivative + feedforward`. Feedforward is `kF×setpoint + kS×sign(setpoint)`, for `update*` only.
6. **Conditional integration.** If that output is past a constrain limit *and* step 2 pushed it further out, I is rolled back (step 3's damping still applies) and the output is recomputed.
7. **Settle tracking** for `atSetpoint()`.
8. **Shaping.** `setMinOutput()` lifts small nonzero outputs (not inside the tolerance band). `setOutputRampRate()` limits the change from the previous output.
9. **Clamp** to the constrain limits and return.

All internal math is `float`. All inputs and outputs are in *your* units; dt is handled internally in seconds.

---

## Constructor

```cpp
PIDEasy(float kp = 0.0, float ki = 0.0, float kd = 0.0);
PID     myPid(1.0, 0.5, 0.1);   // same class; PID is an alias
```

| State | Default |
|---|---|
| Output constrain | −255 … +255 |
| I-term bound | ± output range width (always on) |
| Raw windup clamp (`setWindUP`) | off |
| Integral limit, output units | off |
| Conditional integration | **on** |
| Derivative smoothing | off (fixed-coefficient mode, 0) |
| Damping factor | 1.0 (no damping) |
| Max internal dt | **500 ms** |
| Continuous input | off |
| Tolerance / `atSetpoint()` | not set (`atSetpoint()` is false) |
| Feedforward, min output, ramp rate | off |

### Name clashes (`PIDEASY_NO_PID_ALIAS`)
`PID_v1` and some other libraries also define a class named `PID`. To use one of them in the same sketch:

```cpp
#define PIDEASY_NO_PID_ALIAS
#include <PIDEasy.h>
#include <PID_v1.h>

PIDEasy heading(2.0, 0.0, 0.3);
```

## Compute functions — pick one per controller

| Call | dt | Derivative of | Use when |
|---|---|---|---|
| `compute(error)` | measured with `micros()` | error | you already have an error value |
| `computeMs(error, dt_ms)` | milliseconds, you measure | error | fixed-rate loops timed with `millis()` |
| `computeUs(error, dt_us)` | microseconds, you measure | error | very fast loops |
| `computeSeconds(error, dt_s)` | float seconds, you measure | error | you already have dt in seconds |
| `update(setpoint, measurement)` | measured with `micros()` | measurement | **turns, wall distance, speed — anything with a setpoint** |
| `updateMs` / `updateUs` / `updateSeconds` | as above | measurement | same, with your own dt |
| `compute(error, dt)` *(deprecated)* | **whole seconds** | error | never on a robot |

All return the output (also available as `getOutput()`).

### `float compute(float error)` / `float update(float setpoint, float measurement)` — internal timer
The two share one `micros()` timer, kept as 32-bit values so the ~71-minute rollover is handled on every core. The first call after construction or `reset()` only starts the timer: it counts as a resume, so it neither integrates nor differentiates. A call before the clock has ticked returns the previous output unchanged. `micros()` has a 4 µs resolution on 16 MHz AVR.

If the gap since the previous call exceeds `setMaxDeltaTime()` (default 500 ms), the sample is a **resume**. The integral is left untouched and the derivative is 0 for that call. Normal behavior returns on the next sample. `reset()` after a planned pause is still cleaner if you also want the integral cleared.

### `float computeMs(float error, unsigned long dt_ms)`
`dt_ms == 0` is treated as 1 ms (unchanged from 1.x). Never a resume: you own the timing.

### `float computeUs(float error, unsigned long dt_us)` / `float computeSeconds(float error, float dt_s)` *(new in 1.2.0)*
`dt == 0` (or negative or NaN for seconds) returns the previous output unchanged.

### `updateMs` / `updateUs` / `updateSeconds(setpoint, measurement, dt)` *(new in 1.2.0)*
Same timing as the matching `compute*`, except `updateMs(…, 0)` holds instead of assuming 1 ms.

### Why `update()` over `compute(error)`
With `compute(error)`, a new target changes the error in one step. The D-term then spikes: going from heading 0 to target 90 with `kd = 0.5` on a 10 ms loop gives a D-term of **4500** for one cycle. `update()` differentiates only the measurement, so the same step gives 0. With a fixed setpoint the two give identical results.

Switching one object between `compute*` and `update*` is allowed. The derivative is 0 for the first sample after a switch.

### `float compute(float error, unsigned long dt)` — deprecated, dt in **whole SECONDS**
Kept for sketches written for the original PIDEasy. `dt == 0` counts as **1 second**, and any real loop period truncates to 0. The integral then grows 10–100× too fast and the derivative is 10–100× too weak. If robot code calls this with a millis-based dt, that is a bug: use `computeMs()`.

## State

### `void reset()`
Clears the I-term, derivative history, internal timer, telemetry, and settle state. Gains and settings are kept. Call it before a new motion (a turn, a new corridor) and after a pause.

### `void resetIntegral()` *(new in 1.2.0)*
Clears only the I-term; the derivative history and timer are kept.

### `void setIntegral(float value)` *(new in 1.2.0)*
Preloads the I-term in output units, e.g. the PWM an arm needs to hold its weight. Clamped by the integral limits. NaN is ignored.

## Gains

### `void setTunings(float kp, float ki, float kd)`
Replaces the gains at runtime. **State is preserved and the switch is bumpless:** the I-term is stored in output units, so a new `ki` applies to future error only. Exception: `ki = 0` clears the I-term, because a frozen I-term would otherwise act as a hidden constant offset. Use one object and `setTunings()` per mode, not a new object (which discards the timer and the I-term).

### `float getKp() const` / `getKi()` / `getKd()`
Current gains.

## Integral / anti-windup

### `void setConditionalIntegration(bool enabled)`
On by default. While the output is past a constrain limit, a step that would push it further out is rolled back. Steps that unwind the saturation are never blocked. This fixes "the robot clips the corner, then overshoots coming out" when the motors sit at ±255 through a curve.

Only the constrain limits count as saturation, not `setOutputRampRate()`. While a ramp holds the output back, the I-term keeps integrating. Keep ramps short, or bound I with `setIntegralLimit()`.

### `void setIntegralLimit(float min, float max)`
Clamps the I-term's contribution to the output, in output units. This keeps its meaning when `ki` changes. Arguments are swapped if reversed. `(0, 0)` disables it.

```cpp
myPID.setIntegralLimit(-60, 60);  // I may never contribute more than ±60
```

### `void setWindUP(float min, float max)`
Legacy clamp on the **raw** integral (error × seconds). Its effect on the output is `ki × limit`, so it shifts whenever `ki` is retuned. Off by default since 1.2.0. Once called, it stays on for that object. Arguments are swapped if reversed. Prefer `setIntegralLimit()`.

### `void setDampingFactor(float dF)`
When the error **crosses zero** (strict sign change between calls), I is multiplied by `dF`, clamped to [0, 1]. 1 = keep (default), 0 = wipe at every crossing. With a noisy sensor near zero error the sign flips often, which suppresses I near the setpoint.

## Output

### `void setConstrain(float min, float max)`
Output limits (default ±255). Arguments are swapped if reversed. Also bounds the I-term to ± the range width.

### `void setMinOutput(float minOutput)` *(new in 1.2.0)*
Motors often do not move below some PWM (~30–50), so a PID near its target stalls short of it. Nonzero outputs smaller than `minOutput` are raised to ±`minOutput`, **except while |error| is within `setTolerance()`'s error tolerance**. Without that band the robot hunts back and forth across the target, so pair the two. 0 disables.

### `void setOutputRampRate(float unitsPerSecond)` *(new in 1.2.0)*
Limits how fast the output may change: `1000` takes a 0…255 output from 0 to full in ~0.26 s. This prevents wheel spin, tipping, and brown-out resets from motor inrush. The first internally timed sample after `reset()` has no elapsed time, so the output soft-starts from 0. 0 disables.

### `void setFeedforward(float kF, float kS = 0)` *(new in 1.2.0)*
For `update*` only: adds `kF × setpoint + kS × sign(setpoint)`. For motor speed, `kS` is the PWM that just overcomes friction and `kF` is PWM per unit of speed above that. The PI loop then only corrects the remainder. It has no effect on `compute*`, which has no setpoint. `(0, 0)` disables (the default).

## Derivative

### `void setDerivativeTimeConstant(float tauSeconds)`
Low-pass filters the derivative with a time constant in seconds. The coefficient `tau / (tau + dt)` is recomputed each call, so smoothing is the same whatever the loop rate. Measured: `tau = 0.1` gives 85.1 at a 10 ms loop and 80.2 at 50 ms, against an analytic 86.5. Start near 2–3× your loop period. 0 disables. It is mutually exclusive with `setSmoothingDerivative()`: the last call wins.

### `void setSmoothingDerivative(float sD)` / alias `setSmoothingDerivate(float sD)`
The same filter with a **fixed** coefficient in [0, 1]. Its effect scales with the loop period: `sD = 0.9` gives 87.8 at 10 ms and 34.4 at 50 ms. Prefer the time constant. To convert: `tau = sD / (1 − sD) × dt`.

## Circular inputs

### `void setContinuousInput(float min, float max)` / `void disableContinuousInput()` *(new in 1.2.0)*
For gyro headings and other angles. The error, and the derivative's difference, are wrapped into ± half of `max − min`. Heading 350 → target 10 is **+20**, not −340. A 179 → −179 step is a 2° move, not 358°. Only the size of the range matters: `(-180, 180)` and `(0, 360)` behave the same. Works for accumulated headings of any size.

## Finished yet?

### `void setTolerance(float errorTol, float rateTol = -1, unsigned long settleMs = 0)` / `bool atSetpoint() const` *(new in 1.2.0)*
`atSetpoint()` is true once **both** of these have held continuously for `settleMs`:
- `|error| ≤ errorTol`
- `|derivative| ≤ rateTol`: the filtered derivative, in error units per second. Negative = not checked.

It is always false before `setTolerance()` and after `reset()`. A resume sample neither adds to nor breaks the settle time.

```cpp
pid.setTolerance(2.0, 15.0, 100);   // within 2°, slower than 15°/s, for 100 ms
pid.reset();
while (!pid.atSetpoint() && millis() - start < 3000) {
  float p = pid.update(target, heading());
  drive(p, -p);
}
```

The rate check matters: a robot sweeping through the target at speed is inside the error band for a moment but not finished.

## Timing

### `void setMaxDeltaTime(unsigned long maxDtMs)`
Longest gap between internally timed calls that still counts as a normal cycle (default 500 ms, 0 disables). A longer gap produces a resume sample. It does not affect the variants where you pass dt.

### `float getDeltaTime() const` / `bool wasResumed() const` *(new in 1.2.0)*
The dt (seconds) used by the last sample that ran, and whether that sample was a resume. Held calls change neither. **If `wasResumed()` is true on every loop, the loop is slower than `setMaxDeltaTime()` and I and D never act.**

## Telemetry

### `float getP() const` / `getI()` / `getD()` / `getF()` / `getOutput()`
Terms from the **last** call: `kp×error`, the I-term, `kd×derivative`, and the feedforward. `getP() + getI() + getD() + getF()` is the output before min-output, ramp and clamp; `getOutput()` is what was returned. All are 0 after `reset()`.

```cpp
Serial.print(pid.getP()); Serial.print('\t');
Serial.print(pid.getI()); Serial.print('\t');
Serial.print(pid.getD()); Serial.print('\t');
Serial.println(pid.getOutput());
```

Print at 115200 baud, and not every loop: at 9600 baud a single line can block the loop for tens of milliseconds.

---

## Cost on an Arduino Uno

Measured in simavr on an ATmega328P at 16 MHz (no FPU), `-Os`:

| Call | Cycles | Time |
|---|---|---|
| `computeMs()` PI only (`kd = 0`) | ~2,600 | ~165 µs |
| `computeMs()` PID | ~3,450 | ~215 µs |
| `computeMs()` PID + tau filter + integral limit | ~4,600 | ~290 µs |
| `updateMs()` with every 1.2.0 feature on | ~7,350 | ~460 µs |

RAM: 147 bytes per controller on AVR. Boards with an FPU (ESP32, Teensy 4, RP2350, STM32F4) are far faster.

---

## Host tests

`extras/test/` holds a regression suite that builds and runs on a desktop compiler against a fake 32-bit `micros()` clock. No board is required:

```bash
./extras/test/run_tests.sh          # or .\extras\test\run_tests.ps1 on Windows
```

102 checks. Non-zero exit on failure. CI also runs them under AddressSanitizer/UBSan and clang, runs `arduino-lint`, and compiles the examples for Uno, Mega, Raspberry Pi Pico and ESP32. See `extras/test/README.md`.

---

## Checklist for auditing robot code

- [ ] **Never** calls the deprecated two-argument `compute(error, dt)` with a real loop dt — use `computeMs()` / `computeSeconds()`.
- [ ] Controllers with a setpoint (turn to heading, wall distance, speed) use `update(setpoint, measurement)`, not `compute(setpoint - measurement)`, so a new target does not kick the D-term.
- [ ] Heading controllers call `setContinuousInput(-180, 180)` (or `(0, 360)`).
- [ ] "Turn finished" uses `setTolerance(…)` + `atSetpoint()` **with a timeout**, and includes a rate tolerance so the robot doesn't count sweeping through the target as done.
- [ ] `setMinOutput()` is paired with `setTolerance()` so the robot does not hunt around the target.
- [ ] Calls `reset()` before a new controlled motion and after planned pauses.
- [ ] A lost line / sensor timeout (NaN) is handled by the robot logic: the PID holds its last output, it does not stop the motors.
- [ ] `wasResumed()` is not true on every loop (loop slower than `setMaxDeltaTime()`).
- [ ] Integral bounded with `setIntegralLimit()` (output units) rather than `setWindUP()`.
- [ ] Conditional integration left on unless there is a measured reason to turn it off.
- [ ] Mode switches use `setTunings()` on one object, not a freshly constructed one.
- [ ] Derivative filtering uses `setDerivativeTimeConstant()` rather than `setSmoothingDerivative()` if the loop period varies.
- [ ] Error sign convention is consistent: `update()` uses `setpoint − measurement`.
- [ ] One object per controlled quantity (heading, wall distance, each wheel's speed).
