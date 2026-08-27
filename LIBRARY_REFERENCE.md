# PIDEasy-Improved — Library Reference

Reference for auditing robot code that uses this library (RoboCup Junior Rescue Maze).
Covers version 1.1.0. One class: `PID`, declared in `src/PIDEasy.h`, implemented in `src/PIDEasy.cpp`.

## ⚠️ Behavior changes in 1.1.0

All existing method signatures are unchanged and old sketches still compile, but two defaults now behave differently:

1. **Conditional integration is ON by default.** When the output is already outside the constrain limits and the current step would push it further out, the integration step is rolled back. Previously the integral kept charging while the motors were saturated and then dumped as overshoot. To restore the old behavior: `setConditionalIntegration(false)`.
2. **`setMaxDeltaTime()` now defaults to 100 ms (was 1000 ms), and an over-cap gap is treated as a *resume*** — the integral and derivative are skipped for that one sample instead of taking a full clamped step. To restore something close to the old behavior: `setMaxDeltaTime(1000)`.

Both changes only affect the one-argument `compute(error)` (change 2) and any call where the output saturates (change 1). If you had tuned around the old windup behavior, re-check your gains.

## Quick model of how it works

Each `compute*` call does, in order:

1. `integral += error * dt` (dt internally in **seconds**), then clamps `integral` to the windup limits and, if enabled, to the output-unit integral limit. **Skipped entirely on a resume sample.**
2. If `error` changed sign versus the previous call (strictly positive → strictly negative or vice versa), multiplies `integral` by `dampingFactor`.
3. Derivative = `(error - previous_error) / dt`, **forced to 0 on the very first sample** after construction or `reset()`, and on a resume sample (avoids derivative kick). Then low-pass filtered: `d = smoothing * previous_d + (1 - smoothing) * d`, where `smoothing` is either the fixed coefficient from `setSmoothingDerivative()` or `tau / (tau + dt)` when `setDerivativeTimeConstant()` is in use.
4. Output = `kp*error + ki*integral + kd*derivative`.
5. **Conditional integration:** if that output is past a constrain limit *and* step 1 pushed it further out, the integral is rolled back to its pre-step value (damping from step 2 still applied) and the output is recomputed.
6. Output clamped to the constrain limits and returned.
7. Stores `previous_error`, `previous_derivative`, and the per-term contributions for the getters.

All internal math is `float`.

---

## Constructor

```cpp
PID(float kp = 0.0, float ki = 0.0, float kd = 0.0);
```

Defaults set by the constructor:

| State | Default |
|---|---|
| Windup limits (integral clamp) | −255 … +255 |
| Output constrain | −255 … +255 |
| Integral limit (output units) | disabled |
| Conditional integration | **enabled** |
| Derivative smoothing | 0 (no smoothing, fixed-coefficient mode) |
| Damping factor | 1.0 (no damping) |
| Max internal dt | **100 ms** |

## Compute functions (three variants — dt units differ!)

### `float compute(float error, unsigned long dt)` — dt in **SECONDS**
Backwards-compatible with the original PIDEasy. `dt` is an **integer number of seconds**; `dt == 0` is treated as **1 second**.

⚠️ **Trap:** in a typical robot loop that runs every 10–100 ms, any dt you can pass here truncates to 0 and becomes **1 full second**. That makes the integral accumulate ~10–100× too fast and the derivative ~10–100× too weak. **Do not use this variant on the robot.** If robot code calls the two-argument `compute()` with a millis-based dt, that is a bug — it should be `computeMs()`.

### `float computeMs(float error, unsigned long dt_ms)` — dt in **milliseconds**
The correct variant when you measure dt yourself. `dt_ms == 0` is treated as 1 ms. Never produces a resume sample — you own the timing here.

### `float compute(float error)` — dt measured internally with `millis()`
First call initializes the internal timer and uses dt = 1 ms (derivative is suppressed anyway on the first sample). Subsequent calls use elapsed `millis()`. Handles `millis()` rollover correctly (unsigned subtraction).

If the measured gap exceeds `setMaxDeltaTime()` (default 100 ms), the sample is treated as a **resume**: the integral is left untouched and the derivative is forced to 0 for that call. Normal behavior returns on the next in-window sample. Calling `reset()` after a pause is still the cleaner option when you also want the accumulated integral cleared.

⚠️ **Trap:** if the loop runs **faster than 1 kHz**, dt clamps to 1 ms while real dt is shorter → derivative is over-estimated and jittery. Add a small delay or use `computeMs` with `micros()`-derived timing if your loop is that fast.

## `void reset()`
Clears integral, previous error, previous derivative, the first-sample flag, the internal `millis()` timer, and the telemetry getters. **Call this whenever the setpoint changes discontinuously** (start of a turn, new wall-follow segment, after a pause). Because the derivative acts on *error*, a sudden setpoint change otherwise produces a one-cycle derivative kick.

## `void setTunings(float kp, float ki, float kd)` *(new in 1.1.0)*
Replaces all three gains at runtime. **Internal state is deliberately preserved** — integral, derivative history, and the `millis()` timer all survive, so a mode switch (line follow → gap → turn) is bumpless. Call `reset()` alongside it if you *want* the state cleared.

Re-applies the output-unit integral limit, since that limit is defined relative to `ki`.

## `float getKp()` / `float getKi()` / `float getKd()` *(new in 1.1.0)*
Current gains.

## `void setWindUP(float min, float max)`
Clamp range for the **raw integral** (anti-windup). The clamp applies before multiplication by `ki`, so the I-term's max contribution to output is `ki * max_windup` — meaning the effective limit changes whenever you retune `ki`. Prefer `setIntegralLimit()` for new code. Arguments are swapped automatically if given in the wrong order.

## `void setIntegralLimit(float min, float max)` *(new in 1.1.0)*
Clamps the integral's **contribution to the output** (`ki * integral`) rather than the raw integral, so the limit keeps its meaning when `ki` is retuned. Applied on top of `setWindUP()` (both are enforced; the tighter one wins).

Only active while `ki > 0` — with `ki == 0` the I-term contributes nothing anyway, and a negative `ki` would flip the interval. Arguments are swapped automatically if given in the wrong order. Pass `(0, 0)` to disable and fall back to `setWindUP()` alone.

```cpp
myPID.setIntegralLimit(-60, 60);  // I may never contribute more than ±60 of the ±255 output
```

## `void setConditionalIntegration(bool enabled)` *(new in 1.1.0)*
Enabled by default. When on, the integration step is rolled back if the output is already past a constrain limit and that step pushed it further out. Integration that *unwinds* saturation is never blocked, so recovery is not delayed.

This is the fix for the classic "robot clips the corner then overshoots coming out" symptom, where the motors sit at ±255 through a sharp curve. Turn it off only to A/B compare during tuning.

## `void setMaxDeltaTime(unsigned long maxDtMs)`
Caps the dt measured internally by the one-argument `compute(error)`. **Default 100 ms since 1.1.0** (was 1000 ms); pass 0 to disable the cap. A gap over the cap produces a resume sample (see `compute(error)` above). Does not affect `computeMs()` or the seconds-based `compute()`, where you supply dt yourself.

## `void setSmoothingDerivative(float sD)` / alias `setSmoothingDerivate(float sD)`
Exponential low-pass filter on the derivative term, with a **fixed coefficient**. `sD` is clamped to [0, 1]. 0 = no smoothing (default), values near 1 = heavy smoothing (more of the previous derivative, more lag). `setSmoothingDerivate` (misspelled) is a backwards-compatible alias — identical behavior.

⚠️ **Trap:** the coefficient is fixed, so the filter's effective time constant scales with your loop period. If the loop rate jitters (SD writes, LCD updates, slow ToF reads), the amount of smoothing jitters with it. Measured: driving the same ramp for 200 ms of wall clock with `sD = 0.9` yields a filtered derivative of **87.8 at a 10 ms loop but 34.4 at 50 ms** — a 2.5× difference from loop timing alone. Prefer `setDerivativeTimeConstant()` on any robot whose loop period is not steady.

## `void setDerivativeTimeConstant(float tauSeconds)` *(new in 1.1.0)*
Same low-pass filter, but specified as a **time constant in seconds**. The coefficient becomes `tau / (tau + dt)`, recomputed from the measured `dt` on every call, so the amount of smoothing stays constant as the loop period moves. Over the same test as above, `tau = 0.1` yields **85.1 at a 10 ms loop and 80.2 at 50 ms**, both close to the analytic step response of 86.5.

Pass 0 (or a negative value) to turn derivative filtering off entirely.

Mutually exclusive with `setSmoothingDerivative()` — whichever was called last wins, in both directions.

Picking a value: `tau` is roughly the time the filter takes to reach 63% of a step in the derivative. Start near 2–3× your nominal loop period (e.g. `0.05` for a 20 ms loop) and raise it if D is still noisy. To convert an existing fixed coefficient you already like: `tau = sD / (1 - sD) * dt`, using the loop period you tuned it at — `sD = 0.8` at a 20 ms loop is `tau = 0.08`.

## `void setDampingFactor(float dF)`
When the error **crosses zero** (strict sign change between consecutive calls), the integral is multiplied by `dF`. Values in **[0, 1]**: 1 = keep integral (default), 0 = wipe integral at every crossing, 0.5 = halve it. Since 1.0.6 the value is clamped to [0, 1].

Note: with a noisy sensor hovering near zero error, the sign flips often and the integral is damped at every flip — this effectively suppresses the I-term near the setpoint. Usually harmless for wall following, but be aware if you rely on I to remove steady-state offset.

## `float getP()` / `float getI()` / `float getD()` / `float getOutput()` *(new in 1.1.0)*
Per-term contributions from the **last** `compute*()` call: `kp*error`, `ki*integral`, and `kd*derivative` respectively, after any conditional-integration rollback. `getP() + getI() + getD()` is the output *before* the constrain clamp; `getOutput()` is the value actually returned.

Intended for tuning telemetry — printing the three terms tells you at a glance whether I is winding up or D is just amplifying sensor noise:

```cpp
float out = myPID.compute(error);
Serial.print(myPID.getP()); Serial.print('\t');
Serial.print(myPID.getI()); Serial.print('\t');
Serial.print(myPID.getD()); Serial.print('\t');
Serial.println(myPID.getOutput());
```

All four return 0 after `reset()` and before the first compute call.

---

## Host tests

`extras/test/` holds a regression suite that builds and runs on a desktop compiler against a fake `millis()` clock — no board required:

```bash
./extras/test/run_tests.sh          # or .\extras\test\run_tests.ps1 on Windows
```

36 checks covering every method, including two things that are impractical to verify on hardware: `millis()` rollover (once per ~49 days of uptime) and the loop-rate independence of the derivative filter. Non-zero exit on failure. `extras/` is ignored by the Arduino build system, so none of it reaches the board. See `extras/test/README.md`.

Run it after any change to `src/`, and before a competition.

---

## Checklist for auditing robot code

- [ ] **Never** calls the two-argument `compute(error, dt)` with milliseconds — use `computeMs()` (this is the single most damaging misuse).
- [ ] Calls `reset()` after pauses (victim stop, kit drop) and before starting a new controlled motion (turn, new corridor) when using the `compute(error)` millis-based variant.
- [ ] Integral bounded by `setIntegralLimit()` (output units) rather than a raw `setWindUP()` value that silently changes meaning when `ki` is retuned.
- [ ] Conditional integration left enabled unless there is a measured reason to disable it.
- [ ] Gain switching between modes uses `setTunings()` on one object, not a freshly constructed `PID` (which silently discards the integral and timer).
- [ ] Loop rate is ≥ ~1 ms per iteration if using `compute(error)`, and the typical loop period is comfortably under `setMaxDeltaTime()` — otherwise normal cycles get misread as resumes and the I-term never accumulates.
- [ ] Derivative filtering uses `setDerivativeTimeConstant()` rather than `setSmoothingDerivative()` if the loop period is not steady (SD writes, LCD updates, slow ToF reads).
- [ ] Error convention is consistent: the library computes on the error you give it; sign of the output depends on your `error = measurement − setpoint` vs `setpoint − measurement` choice.
- [ ] One `PID` object per controlled quantity (heading, wall distance, etc.) — state (integral, previous error, internal timer) is per-object.
