#ifndef PIDEASY_H
#define PIDEASY_H

#include <Arduino.h>
#include <stdint.h>

#if defined(__GNUC__)
#define PIDEASY_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
#define PIDEASY_DEPRECATED(msg)
#endif

class PID {
  public:
    // Constructor: Kp, Ki, Kd
    PID(float kp = 0.0, float ki = 0.0, float kd = 0.0);

    // (original library used seconds). Use this to avoid breaking existing sketches.
    // dt is a whole number of seconds and 0 counts as 1 s, so any dt from a
    // real robot loop is wrong here. Use computeMs() or computeSeconds().
    PIDEASY_DEPRECATED("dt is whole seconds; use computeMs() or computeSeconds()")
    float compute(float error, unsigned long dt);

    // Pass `dt` in seconds as a float (e.g. 0.02 for a 20 ms loop).
    // dt <= 0 (or NaN) returns the last output and updates nothing.
    float computeSeconds(float error, float dt_s);

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
    float updateSeconds(float setpoint, float measurement, float dt_s);

    void reset();

    // Clear only the I-term, keeping the derivative history and timer.
    void resetIntegral();

    // Preload the I-term, in output units (e.g. the output a lift needs to
    // hold its weight). Clamped by the integral limits.
    void setIntegral(float value);

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

    // Treat the input as circular, e.g. a gyro heading in degrees with
    // (-180, 180) or (0, 360). The error is wrapped into half a turn either
    // way, so going from 350 to 10 degrees turns 20 degrees, not 340. Also
    // applies to the derivative. The range is max - min; only its size
    // matters. disableContinuousInput() turns it off (the default).
    void setContinuousInput(float min, float max);
    void disableContinuousInput();

    // Define "on target" for atSetpoint(). errorTolerance is in error
    // units; rateTolerance is in error units per second and is checked
    // against the filtered derivative (pass a negative value to ignore it);
    // settleMs is how long both must hold continuously. For example, a
    // turn is finished at setTolerance(2.0, 10.0, 100): within 2 degrees,
    // turning slower than 10 deg/s, for 100 ms.
    void setTolerance(float errorTolerance, float rateTolerance = -1.0f,
                      unsigned long settleMs = 0);

    // True when the last samples satisfied setTolerance(). Always false
    // until setTolerance() has been called, and after reset().
    bool atSetpoint();

    // Feedforward for the setpoint / measurement variants (update*()):
    // adds kF * setpoint + kS * sign(setpoint) to the output. For motor
    // speed control, kF is roughly (output per unit of speed) and kS is the
    // output needed to overcome static friction. No effect on compute*(),
    // which has no setpoint. Pass (0, 0) to disable (the default).
    void setFeedforward(float kF, float kS = 0.0f);

    // Motors often do not move below some output (PWM ~30-50). Raise any
    // nonzero output smaller than minOutput to +/-minOutput, except while
    // the error is inside setTolerance()'s errorTolerance, so the robot does
    // not hunt around the target. Pair it with setTolerance(). 0 disables.
    void setMinOutput(float minOutput);

    // Limit how fast the output may change, in output units per second
    // (e.g. 1000 lets a 255 PWM output go from 0 to full in ~0.26 s).
    // Prevents wheel slip, tipping, and brown-out resets from motor inrush.
    // Also soft-starts from 0 after reset(). 0 disables (the default).
    void setOutputRampRate(float unitsPerSecond);

    // Longest gap between compute(error) / update() calls that still counts
    // as a normal loop cycle (milliseconds). A longer gap (e.g. a stop to
    // signal a victim) is treated as a resume: that one sample skips the
    // integral and derivative. Default 500 ms. Pass 0 to disable.
    void setMaxDeltaTime(unsigned long maxDtMs);

    // Last computed contribution of each term, for tuning telemetry.
    // getP() + getI() + getD() + getF() is the output before
    // setMinOutput(), setOutputRampRate() and the constrain clamp;
    // getOutput() is the value actually returned by the last compute*().
    float getP();
    float getI();
    float getD();
    float getOutput();
    // Feedforward contribution of the last update*() call.
    float getF();

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
               float dt, bool resume, float feedforward);

    // Feedforward and error for the setpoint / measurement variants.
    float updateInternal(float setpoint, float measurement, float dt, bool resume);

    // Wrap a difference into half the continuous range either way.
    float wrap(float x);

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
    float continuous_min, continuous_max;
    bool continuous_enabled;

    float error_tolerance, rate_tolerance, settle_s;
    bool tolerance_enabled;
    bool in_tolerance;
    // Time the tolerance has held continuously, in seconds.
    float settled_time;

    float kf, ks, last_f;
    float min_output;
    float ramp_rate;

    uint32_t lastMicros;
    bool hasLastMicros;
    unsigned long max_dt_ms;
};

#endif
