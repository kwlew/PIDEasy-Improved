#ifndef PIDEASY_H
#define PIDEASY_H

#include <Arduino.h>
#include <stdint.h>

class PID {
  public:
    // Constructor: Kp, Ki, Kd
    PID(float kp = 0.0, float ki = 0.0, float kd = 0.0);

    // (original library used seconds). Use this to avoid breaking existing sketches.
    float compute(float error, unsigned long dt);

    // New Variant: pass `dt` in milliseconds.
    float computeMs(float error, unsigned long dt_ms);

    // Pass `dt` in microseconds, for loops too fast for millisecond timing.
    // dt_us == 0 means no time has passed: the last output is returned and
    // nothing is updated.
    float computeUs(float error, unsigned long dt_us);

    // Measures dt internally with micros(). The first call only starts the
    // timer; see setMaxDeltaTime() for how long pauses are handled.
    float compute(float error);

    // Setpoint / measurement variants. The error is setpoint - measurement,
    // but the derivative is taken on the measurement alone, so changing the
    // setpoint (a new turn target, a new wall distance) does not produce a
    // derivative kick. Same timing rules as the matching compute*() call;
    // dt == 0 returns the last output unchanged.
    float update(float setpoint, float measurement);
    float updateMs(float setpoint, float measurement, unsigned long dt_ms);
    float updateUs(float setpoint, float measurement, unsigned long dt_us);

    void reset();

    // Change the gains at runtime without losing the integral or the
    // derivative/timer history. Useful when a robot switches modes
    // (line follow / gap / turn) and each mode needs different tuning.
    // The I-term is stored in output units, so changing ki does not make
    // the output jump. Setting ki to 0 clears the I-term.
    void setTunings(float kp, float ki, float kd);

    float getKp();
    float getKi();
    float getKd();

    // Clamp the raw integral (error x seconds) to [min, max]. Its effect on
    // the output is ki * limit, so it changes whenever ki is retuned; prefer
    // setIntegralLimit(). Off by default: the I-term is always kept inside
    // the setConstrain() range, which is enough for most robots.
    void setWindUP(float min, float max);

    // Limit the integral's *contribution to the output* (ki * integral)
    // instead of the raw integral, so the limit keeps its meaning when ki
    // is retuned. Applied on top of setWindUP().
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

    // Longest gap between compute(error) / update() calls that still counts
    // as a normal loop cycle (milliseconds). A longer gap (e.g. a stop to
    // signal a victim) is treated as a resume: that one sample skips the
    // integral and derivative. Default 500 ms. Pass 0 to disable.
    void setMaxDeltaTime(unsigned long maxDtMs);

    // Last computed contribution of each term, for tuning telemetry.
    // getP() + getI() + getD() is the output before the constrain clamp;
    // getOutput() is the value actually returned by the last compute*().
    float getP();
    float getI();
    float getD();
    float getOutput();

    // Timing diagnostics for the last sample that ran. getDeltaTime() is the
    // dt it used, in seconds; wasResumed() is true when it was treated as a
    // resume (first sample, or a gap longer than setMaxDeltaTime()). If
    // wasResumed() is true on every loop, the loop is slower than the cap
    // and the I and D terms never act.
    float getDeltaTime();
    bool wasResumed();

  private:
    // Where the derivative is taken from.
    enum DerivativeSource { SOURCE_ERROR, SOURCE_MEASUREMENT };

    // Shared implementation. dt is in seconds. `dInput` is the quantity the
    // derivative is taken of (the error, or minus the measurement). `resume`
    // marks a sample without meaningful elapsed time: the integral and
    // derivative are skipped.
    float step(float error, float dInput, DerivativeSource source,
               float dt, bool resume);

    // Measure elapsed time with micros(). Returns false when no time has
    // passed since the previous call, in which case the caller holds.
    bool measureTime(float* dt, bool* resume);

    // Clamp the I-term to the output range, the windup limits and, if
    // enabled, the output-unit integral limit.
    void applyIntegralLimits();

    float kp, ki, kd;
    float previous_error;
    float previous_input;
    float previous_derivative;
    // I-term in output units (the sum of ki * error * dt).
    float i_term;
    float min_windup, max_windup;
    bool windup_enabled;
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

    // Last per-term contributions (kp*error, I-term, kd*derivative)
    // and the last constrained output.
    float last_p, last_i, last_d, last_output;
    float last_dt;
    bool last_resumed;

    // True once a valid previous_input exists; used to suppress the
    // derivative term on the first sample (avoids a derivative "kick").
    bool hasPreviousInput;
    DerivativeSource previous_source;

    // For the internally timed compute(error) / update().
    uint32_t lastMicros;
    bool hasLastMicros;
    unsigned long max_dt_ms;
};

#endif
