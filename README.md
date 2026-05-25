# PIDEasy-Improved - PID Controller library for Arduino based environments.

## Original PIDEasy: https://github.com/vsjoaopedrovs/PIDEasy
This library is a fork of the original PIDEasy library.

## 🚀 Features
- Simple and lightweight
- Supports windup limits
- Includes derivative smoothing
- Allows output constraints

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

### 7️⃣ Adjust Damping Factor (Optional)
```cpp
myPID.setDampingFactor(0.8);
```

### 8️⃣ Check example for more info.

## 📜 License
This project is licensed under the MIT License.

## 🤝 Contributing
Feel free to contribute! Fork the repository and submit a pull request.

