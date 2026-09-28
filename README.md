# PIDEasy-Improved - PID Controller library for Arduino based environments.

## Original PIDEasy: https://github.com/vsjoaopedrovs/PIDEasy
This library is a fork of the original PIDEasy library.

A small PID library aimed at competition robots (RoboCup Junior, WRO, line followers, sumo): the parts every robot ends up writing by hand around a PID are built in and tested.

## 🚀 Features
- Simple and lightweight, all `float`, no dynamic memory
- Measures `dt` itself with `micros()`, or takes it in seconds, milliseconds or microseconds
- `update(setpoint, measurement)`: derivative on measurement, so a new target doesn't kick the D-term
- Heading wrap-around for gyros (`setContinuousInput`): 350° → 10° turns 20°, not 340°
- "Turn finished?" check (`setTolerance` + `atSetpoint`) with error, rate and settle time
- Feedforward (`kF`, `kS`) for motor speed control
- Motor deadband compensation (`setMinOutput`) and output ramp limiting (`setOutputRampRate`)
- Saturation-aware anti-windup (conditional integration), integral limits in output units
- Bumpless runtime gain changes (`setTunings`)
- Loop-rate-independent derivative filtering
- NaN-safe: a sensor glitch (like a 0/0 line position) holds the output instead of breaking the controller
- Per-term telemetry (`getP`/`getI`/`getD`/`getF`) for tuning over serial
- Host test suite (102 checks) and CI builds for Uno, Mega, Pico and ESP32

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

### PlatformIO
Add to `platformio.ini`:
```ini
lib_deps = https://github.com/kwlew/PIDEasy-Improved.git
```

## 📖 Usage

### 1️⃣ Include the library and create a controller
```cpp
#include <PIDEasy.h>

PIDEasy myPID(1.0, 0.5, 0.1); // Kp, Ki, Kd  (the old name PID still works)
```

### 2️⃣ Compute the output
If you have a setpoint and a measurement (a heading, a distance, a speed), use `update()`:
```cpp
float output = myPID.update(targetHeading, gyroHeading); // dt measured with micros()
```

If you only have an error value (e.g. a line position), use `compute()`:
```cpp
float output = myPID.compute(linePosition); // error; dt measured with micros()
```

Measuring dt yourself? Use `computeMs(error, dt_ms)`, `computeUs(error, dt_us)` or `computeSeconds(error, dt_s)`, and likewise `updateMs` / `updateUs` / `updateSeconds`.

⚠️ The old two-argument `compute(error, dt)` takes dt in **whole seconds**, which is wrong for any robot loop. It is deprecated and kept only for old sketches.

### 3️⃣ Output limits
```cpp
myPID.setConstrain(-255.0, 255.0); // default
```

### 4️⃣ Gyro turns: wrap-around and "finished"
```cpp
turnPID.setContinuousInput(-180, 180);   // headings wrap
turnPID.setTolerance(2.0, 15.0, 100);    // within 2°, slower than 15°/s, for 100 ms
turnPID.setMinOutput(40);                // wheels don't move below ~40 PWM

turnPID.reset();
unsigned long start = millis();
while (!turnPID.atSetpoint() && millis() - start < 3000) {
  float power = turnPID.update(target, readHeading());
  drive(power, -power);
}
```
See `examples/gyroTurn`.

### 5️⃣ Motor speed: feedforward and ramping
```cpp
speedPID.setFeedforward(0.35, 25.0);  // kF * target + kS: most of the PWM, before any error
speedPID.setOutputRampRate(500);      // max 500 PWM per second: no wheel spin, no brown-outs
float pwm = speedPID.updateMs(targetSpeed, measuredSpeed, dt_ms);
```
See `examples/motorSpeedFeedforward` for how to measure `kF` and `kS`.

### 6️⃣ Derivative smoothing
```cpp
myPID.setDerivativeTimeConstant(0.05); // seconds; the same smoothing at any loop rate
```
The older `setSmoothingDerivative(0.8)` (alias `setSmoothingDerivate`) uses a fixed coefficient, so its effect changes with the loop period. To convert: `tau = sD / (1 - sD) * dt`, so `sD = 0.8` on a 20 ms loop becomes `tau = 0.08`.

### 7️⃣ Anti-windup
Conditional integration is on by default: while the output is pinned at a limit, the integral stops charging up. This prevents the overshoot after a hard turn. To cap the I-term further, in output units:
```cpp
myPID.setIntegralLimit(-60.0, 60.0); // I contributes at most ±60
```
`setWindUP(min, max)` (a clamp on the raw integral) still works, but its effect changes whenever you retune `ki`.

### 8️⃣ Change gains at runtime
```cpp
myPID.setTunings(2.0, 0.05, 0.4); // bumpless: the output doesn't jump
```
Call `reset()` as well if you want a clean slate. `resetIntegral()` clears only the I-term.

### 9️⃣ Read the individual terms while tuning
```cpp
Serial.print(myPID.getP()); Serial.print('\t');
Serial.print(myPID.getI()); Serial.print('\t');
Serial.print(myPID.getD()); Serial.print('\t');
Serial.println(myPID.getOutput());
```
Use 115200 baud and print a few times a second, not every loop.

### 🔟 Check your loop timing
```cpp
if (myPID.wasResumed()) { /* this sample counted as a pause */ }
float dt = myPID.getDeltaTime();  // seconds
```
Gaps longer than `setMaxDeltaTime()` (default 500 ms) count as a pause: the I and D terms skip that one sample. If `wasResumed()` is true on every loop, your loop is too slow for the cap.

### Using PID_v1 in the same sketch
```cpp
#define PIDEASY_NO_PID_ALIAS   // frees the name PID for the other library
#include <PIDEasy.h>
#include <PID_v1.h>
```

### Examples
- `lineFollowerExample`: analog sensor array, two motors, surviving a lost line
- `gyroTurn`: turn to a heading, with wrap-around and a settle check
- `motorSpeedFeedforward`: encoder speed control with feedforward and ramping

Full details for every function are in [LIBRARY_REFERENCE.md](LIBRARY_REFERENCE.md).

## 🧪 Running the tests
The controller has a regression suite that builds and runs on your **PC** against a fake
`micros()` clock — no board, no upload. Run it after changing anything in `src/`, and
before a competition:

```powershell
.\extras\test\run_tests.ps1
```

```bash
./extras/test/run_tests.sh
```

102 checks, non-zero exit on failure. Needs `g++` (or any C++ compiler — set `CXX`).
Everything lives under `extras/`, which the Arduino build system ignores, so it never
reaches the board. Details in [extras/test/README.md](extras/test/README.md).

## ⚠️ Upgrading
Every existing sketch still compiles. Behavior changes in **1.2.0**:
- `compute(error)` times itself with `micros()`: dt is accurate at 1–2 kHz, so the D-term is calmer.
- `setTunings()` is bumpless (the output no longer jumps when `ki` changes), and `ki = 0` clears the I-term.
- No raw windup clamp by default. The I-term is bounded by the width of the output range, and conditional integration keeps it in check. A small `ki` is no longer silently capped at `ki × 255`.
- `setMaxDeltaTime()` defaults to 500 ms (was 100 ms).

In **1.1.1**, NaN errors began holding the output, and calls with no elapsed time stopped counting as 1 ms. In **1.1.0**, conditional integration became the default, and long gaps became resumes.

If you tuned your gains around the old behavior, re-check them. See [LIBRARY_REFERENCE.md](LIBRARY_REFERENCE.md) for details.

## 📜 License
This project is licensed under the MIT License.

## 🤝 Contributing
Feel free to contribute! Fork the repository and submit a pull request.
