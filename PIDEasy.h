#ifndef PIDEASY_H
#define PIDEASY_H

#include <Arduino.h>

class PID {
  public:
    // Constructor: Kp, Ki, Kd
    PID(float kp = 0.0, float ki = 0.0, float kd = 0.0);

    // Compute control output.
    // Backwards-compatible Variant A: `compute(error, dt)` expects `dt` in seconds
    // (original library used seconds). Use this to avoid breaking existing sketches.
    float compute(float error, unsigned long dt);

    // New Variant: pass `dt` in milliseconds.
    float computeMs(float error, unsigned long dt_ms);

    // Convenience: compute using `millis()` internally (uses milliseconds).
    float compute(float error);

    void reset();

    // Integral windup limits (floats)
    void setWindUP(float min, float max);

    // Output constraints (floats)
    void setConstrain(float min, float max);

    // Derivative smoothing: value in [0..1].
    // New, correctly spelled API:
    void setSmoothingDerivative(float sD);
    // Backwards-compatible alias (keeps original name)
    void setSmoothingDerivate(float sD);

    void setDampingFactor(float dF);

  private:
    float kp, ki, kd;
    float previous_error;
    float previous_derivative;
    float integral;
    float min_windup, max_windup;
    float derivative_smoothing, dampingFactor;
    float min_constrain, max_constrain;

    // For compute() overload that uses millis()
    unsigned long lastMillis;
    bool hasLastMillis;
};

#endif
