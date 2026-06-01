#ifndef PIDEASY_H
#define PIDEASY_H

#include <Arduino.h>

class PID {
  public:
    // Constructor: Kp, Ki, Kd
    PID(float kp = 0.0, float ki = 0.0, float kd = 0.0);

    // (original library used seconds). Use this to avoid breaking existing sketches.
    float compute(float error, unsigned long dt);

    // New Variant: pass `dt` in milliseconds.
    float computeMs(float error, unsigned long dt_ms);


    float compute(float error);

    float compute(double error);

    void reset();

    void setWindUP(float min, float max);

    void setConstrain(float min, float max);

    void setSmoothingDerivative(float sD);

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
