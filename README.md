# PIDEasy-Improved - PID Controller library for Arduino based environments.

## Original PIDEasy: https://github.com/vsjoaopedrovs/PIDEasy
This library is a fork of the original PIDEasy library.

## 🚀 Features
- Simple and lightweight
- Measures `dt` automatically, or takes it from you in seconds or milliseconds
- Runtime gain changes that keep the controller state (bumpless mode switching)
- Saturation-aware anti-windup (conditional integration), plus windup limits
- Integral limits expressed in output units, so they survive a `ki` retune
- Derivative smoothing, either as a fixed coefficient or as a loop-rate-independent time constant
- Allows output constraints
- Per-term telemetry (`getP`/`getI`/`getD`) for tuning over serial
- Host test suite that runs on your PC — no board needed

## 📥 Installation
### Arduino IDE (Manual Installation)
1. Download the latest version of **PIDEasy-Improved** from the [GitHub Releases](https://github.com/kwlew/PIDEasy-Improved/releases).
2. Extract the ZIP file.
3. Move the `PIDEasy-Improved` folder to your Arduino libraries directory:
   - **Windows:** `Documents/Arduino/libraries`
   - **Mac:** `~/Documents/Arduino/libraries`
   - **Linux:** `~/Arduino/libraries`
4. Restart the Arduino IDE.
5. Go to **Sketch** > **Include Library** > **Manage Libraries**, search for `PIDEasy-Improved`, and check if it's installed.

## 📖 Usage

### 1️⃣ Include the Library
```cpp
#include <PIDEasy.h>
```

### 2️⃣ Create a PID Controller
```cpp
PID myPID(1.0, 0.5, 0.1); // Kp, Ki, Kd
```

### 3️⃣ Compute the PID Output
The library preserves the original API for backwards compatibility.


- Backwards-compatible `compute(error, dt)` expects `dt` in seconds (original behavior). If you measure time with `millis()`, convert to seconds before calling:
```cpp
float error = desiredValue - actualValue;
unsigned long dt_ms = millis() - lastTime;
unsigned long dt_seconds = dt_ms / 1000UL; // integer seconds (original API uses unsigned long seconds)
float control = myPID.compute(error, dt_seconds);
lastTime = millis();
```

- New: if you have `dt` in milliseconds, use `computeMs(error, dt_ms)`:
```cpp
unsigned long dt_ms = millis() - lastTime; // dt in ms
float control = myPID.computeMs(error, dt_ms);
```

- Convenience overload `compute(error)` uses `millis()` internally and calls the millisecond variant:
```cpp
float control = myPID.compute(error); // uses internal millis() to compute dt (ms)
```

### 4️⃣ Set Constraints (Optional)
```cpp
myPID.setConstrain(-255.0, 255.0); // Limit output (float allowed)
```

### 5️⃣ Enable Windup Prevention (Optional)
```cpp
myPID.setWindUP(-255, 255);
```

### 6️⃣ Adjust Derivative Smoothing (Optional)
```cpp
myPID.setSmoothingDerivative(0.8); // preferred name
// or the backwards-compatible alias:
myPID.setSmoothingDerivate(0.8);
```

⚠️ This is a **fixed coefficient**, so how much it actually smooths depends on your loop
period. If your loop time jitters (SD card writes, LCD updates, slow distance sensors),
use a time constant in seconds instead — the coefficient is then recomputed from the
measured `dt` every call and the smoothing stays put:

```cpp
myPID.setDerivativeTimeConstant(0.05); // seconds; 0 disables filtering
```

Converting a fixed value you already like: `tau = sD / (1 - sD) * dt` at the loop period
you tuned it at, so `sD = 0.8` on a 20 ms loop becomes `tau = 0.08`. The two are mutually
exclusive — whichever you call last wins.

### 7️⃣ Adjust Damping Factor (Optional)
```cpp
myPID.setDampingFactor(0.8);
```

### 8️⃣ Change Gains at Runtime (Optional)
`setTunings()` keeps the integral, derivative history, and internal timer, so switching
modes mid-run is bumpless. Call `reset()` as well if you want a clean slate instead.
```cpp
myPID.setTunings(2.0, 0.05, 0.4); // e.g. switching from straight-line to curve gains
float kp = myPID.getKp();         // getKi(), getKd() too
```

### 9️⃣ Bound the Integral in Output Units (Optional)
`setWindUP()` clamps the raw integral, so its effective limit is `ki * windup` and changes
whenever you retune `ki`. `setIntegralLimit()` clamps the I-term's actual contribution
to the output instead:
```cpp
myPID.setIntegralLimit(-60.0, 60.0); // I contributes at most ±60 of the ±255 output
myPID.setIntegralLimit(0, 0);        // disable, fall back to setWindUP()
```

### 🔟 Conditional Integration (On by Default)
While the output is pinned at a constrain limit, the integral no longer keeps charging
up — which is what otherwise causes an overshoot as the robot comes out of a hard turn.
Integration that unwinds the saturation is never blocked.
```cpp
myPID.setConditionalIntegration(false); // opt out to compare during tuning
```

### 1️⃣1️⃣ Read the Individual Terms (Optional)
Useful when tuning over the serial monitor — it shows immediately whether the I-term is
winding up or the D-term is just amplifying sensor noise.
```cpp
float out = myPID.compute(error);
Serial.print(myPID.getP()); Serial.print('\t');
Serial.print(myPID.getI()); Serial.print('\t');
Serial.print(myPID.getD()); Serial.print('\t');
Serial.println(myPID.getOutput());
```

### 1️⃣2️⃣ Check example for more info.

## 🧪 Running the tests
The controller has a regression suite that builds and runs on your **PC** against a fake
`millis()` clock — no board, no upload. Run it after changing anything in `src/`, and
before a competition:

```powershell
.\extras\test\run_tests.ps1
```

```bash
./extras/test/run_tests.sh
```

36 checks, non-zero exit on failure. Needs `g++` (or any C++ compiler — set `CXX`).
Everything lives under `extras/`, which the Arduino build system ignores, so it never
reaches the board. Details in [extras/test/README.md](extras/test/README.md).

## ⚠️ Upgrading from 1.0.x
Every existing signature still works, but two defaults changed in 1.1.0:
- **Conditional integration is on by default** — restore the old behavior with `setConditionalIntegration(false)`.
- **`setMaxDeltaTime()` defaults to 100 ms (was 1000 ms)**, and a gap longer than the cap now
  skips the integral and derivative for that one sample rather than taking a full clamped step.
  Restore roughly the old behavior with `setMaxDeltaTime(1000)`.

If you tuned your gains around the old windup behavior, re-check them.
See [LIBRARY_REFERENCE.md](LIBRARY_REFERENCE.md) for full details.

## 📜 License
This project is licensed under the MIT License.

## 🤝 Contributing
Feel free to contribute! Fork the repository and submit a pull request.

