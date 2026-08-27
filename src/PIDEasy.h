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

    void reset();

    // Change the gains at runtime without losing the integral or the
    // derivative/timer history. Useful when a robot switches modes
    // (line follow / gap / turn) and each mode needs different tuning.
    void setTunings(float kp, float ki, float kd);

    float getKp();
    float getKi();
    float getKd();

    void setWindUP(float min, float max);

    // Limit the integral's *contribution to the output* (ki * integral)
    // instead of the raw integral, so the limit keeps its meaning when ki
    // is retuned. Applied on top of setWindUP(). Only active while ki > 0.
    // Pass (0, 0) to disable and fall back to setWindUP() alone.
    void setIntegralLimit(float min, float max);

    // Conditional integration (on by default): skip the integration step
    // whenever the output is already past a constrain limit and this step
    // would push it further out. Prevents the integral from charging up
    // while the motors are saturated and dumping as overshoot afterwards.
    void setConditionalIntegration(bool enabled);

    void setConstrain(float min, float max);

    void setSmoothingDerivative(float sD);

    // Backwards-compatible alias for setSmoothingDerivative().
    void setSmoothingDerivate(float sD);

    // Low-pass the derivative using a time constant in SECONDS instead of a
    // fixed coefficient. The coefficient is recomputed from the measured dt
    // on every call, so the amount of smoothing stays constant even when the
    // loop period jitters (SD writes, slow sensor reads). Prefer this over
    // setSmoothingDerivative() on a robot whose loop rate is not steady.
    // Pass 0 to turn derivative filtering off. Mutually exclusive with
    // setSmoothingDerivative() — whichever was called last wins.
    void setDerivativeTimeConstant(float tauSeconds);

    void setDampingFactor(float dF);

    // Cap the dt measured internally by compute(error) (milliseconds).
    // Protects against a huge integral step after a pause (e.g. victim
    // signaling). Default 100 ms. Pass 0 to disable the cap.
    void setMaxDeltaTime(unsigned long maxDtMs);

    // Last computed contribution of each term, for tuning telemetry.
    // getP() + getI() + getD() is the output before the constrain clamp;
    // getOutput() is the value actually returned by the last compute*().
    float getP();
    float getI();
    float getD();
    float getOutput();

  private:
    // Shared implementation. `resume` marks a sample that follows a gap
    // longer than max_dt_ms: the integral and derivative are skipped
    // because the elapsed time makes both meaningless.
    float computeInternal(float error, unsigned long dt_ms, bool resume);

    // Clamp the integral to the windup limits and, if enabled, to the
    // output-unit integral limit.
    void applyIntegralLimits();

    float kp, ki, kd;
    float previous_error;
    float previous_derivative;
    float integral;
    float min_windup, max_windup;
    float min_integral_out, max_integral_out;
    bool integral_limit_enabled;
    bool conditional_integration;
    float derivative_smoothing, dampingFactor;

    // Derivative filter time constant in seconds. While use_tau_filter is
    // true the smoothing coefficient is derived from it and dt each call,
    // instead of using the fixed derivative_smoothing value.
    float derivative_tau;
    bool use_tau_filter;

    float min_constrain, max_constrain;

    // Last per-term contributions (kp*error, ki*integral, kd*derivative)
    // and the last constrained output.
    float last_p, last_i, last_d, last_output;

    // True once a valid previous_error exists; used to suppress the
    // derivative term on the first sample (avoids a derivative "kick").
    bool hasPreviousError;

    // For compute() overload that uses millis()
    unsigned long lastMillis;
    bool hasLastMillis;
    unsigned long max_dt_ms;
};

#endif
