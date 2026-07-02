# PIDEasy-Improved — Library Reference

Reference for auditing robot code that uses this library (RoboCup Junior Rescue Maze).
Covers version 1.0.6. One class: `PID`, declared in `src/PIDEasy.h`, implemented in `src/PIDEasy.cpp`.

## Quick model of how it works

Each `compute*` call does, in order:

1. `integral += error * dt` (dt internally in **seconds**), then clamps `integral` to the windup limits.
2. If `error` changed sign versus the previous call (strictly positive → strictly negative or vice versa), multiplies `integral` by `dampingFactor`.
3. Derivative = `(error - previous_error) / dt`, **forced to 0 on the very first sample** after construction or `reset()` (avoids first-sample derivative kick). Then low-pass filtered: `d = smoothing * previous_d + (1 - smoothing) * d`.
4. Output = `kp*error + ki*integral + kd*derivative`, clamped to the constrain limits.
5. Stores `previous_error` and `previous_derivative` for the next call.

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
| Derivative smoothing | 0 (no smoothing) |
| Damping factor | 1.0 (no damping) |

## Compute functions (three variants — dt units differ!)

### `float compute(float error, unsigned long dt)` — dt in **SECONDS**
Backwards-compatible with the original PIDEasy. `dt` is an **integer number of seconds**; `dt == 0` is treated as **1 second**.

⚠️ **Trap:** in a typical robot loop that runs every 10–100 ms, any dt you can pass here truncates to 0 and becomes **1 full second**. That makes the integral accumulate ~10–100× too fast and the derivative ~10–100× too weak. **Do not use this variant on the robot.** If robot code calls the two-argument `compute()` with a millis-based dt, that is a bug — it should be `computeMs()`.

### `float computeMs(float error, unsigned long dt_ms)` — dt in **milliseconds**
The correct variant when you measure dt yourself. `dt_ms == 0` is treated as 1 ms.

### `float compute(float error)` — dt measured internally with `millis()`
First call initializes the internal timer and uses dt = 1 ms (derivative is suppressed anyway on the first sample). Subsequent calls use elapsed `millis()`. Handles `millis()` rollover correctly (unsigned subtraction).

Since 1.0.6, dt measured here is **capped at 1000 ms by default** (configurable via `setMaxDeltaTime()`), so a long pause no longer produces a giant integral step. Still call `reset()` after pauses/turns to clear the accumulated integral and avoid derivative kick.

⚠️ **Trap:** if the loop runs **faster than 1 kHz**, dt clamps to 1 ms while real dt is shorter → derivative is over-estimated and jittery. Add a small delay or use `computeMs` with `micros()`-derived timing if your loop is that fast.

## `void reset()`
Clears integral, previous error, previous derivative, the first-sample flag, and the internal `millis()` timer. **Call this whenever the setpoint changes discontinuously** (start of a turn, new wall-follow segment, after a pause). Because the derivative acts on *error*, a sudden setpoint change otherwise produces a one-cycle derivative kick.

## `void setWindUP(float min, float max)`
Clamp range for the **integral term** (anti-windup). Note the clamp applies to the raw integral, which is later multiplied by `ki` — so the I-term's max contribution to output is `ki * max_windup`. Size the limits accordingly (e.g., with `ki = 0.01`, windup ±255 allows an I contribution of only ±2.55; with large `ki` it can saturate the output). Since 1.0.6, arguments are swapped automatically if given in the wrong order.

## `void setConstrain(float min, float max)`
Clamp range for the **final output**. Since 1.0.6, arguments are swapped automatically if given in the wrong order.

## `void setMaxDeltaTime(unsigned long maxDtMs)` *(new in 1.0.6)*
Caps the dt measured internally by the one-argument `compute(error)`. Default 1000 ms; pass 0 to disable the cap. Does not affect `computeMs()` or the seconds-based `compute()`, where you supply dt yourself.

## `void setSmoothingDerivative(float sD)` / alias `setSmoothingDerivate(float sD)`
Exponential low-pass filter on the derivative term. `sD` is clamped to [0, 1]. 0 = no smoothing (default), values near 1 = heavy smoothing (more of the previous derivative, more lag). `setSmoothingDerivate` (misspelled) is a backwards-compatible alias — identical behavior.

## `void setDampingFactor(float dF)`
When the error **crosses zero** (strict sign change between consecutive calls), the integral is multiplied by `dF`. Values in **[0, 1]**: 1 = keep integral (default), 0 = wipe integral at every crossing, 0.5 = halve it. Since 1.0.6 the value is clamped to [0, 1].

Note: with a noisy sensor hovering near zero error, the sign flips often and the integral is damped at every flip — this effectively suppresses the I-term near the setpoint. Usually harmless for wall following, but be aware if you rely on I to remove steady-state offset.

---

## Checklist for auditing robot code

- [ ] **Never** calls the two-argument `compute(error, dt)` with milliseconds — use `computeMs()` (this is the single most damaging misuse).
- [ ] Calls `reset()` after pauses (victim stop, kit drop) and before starting a new controlled motion (turn, new corridor) when using the `compute(error)` millis-based variant.
- [ ] Windup limits sized against `ki` so the I-term contribution is meaningful but bounded.
- [ ] Loop rate is ≥ ~1 ms per iteration if using `compute(error)`.
- [ ] Error convention is consistent: the library computes on the error you give it; sign of the output depends on your `error = measurement − setpoint` vs `setpoint − measurement` choice.
- [ ] One `PID` object per controlled quantity (heading, wall distance, etc.) — state (integral, previous error, internal timer) is per-object.
