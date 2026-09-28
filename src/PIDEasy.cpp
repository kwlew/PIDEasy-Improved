#include "PIDEasy.h"
#include <math.h>
#include <string.h>

// Constructor to initialize the PID controller.
PIDEasy::PIDEasy(const float kp, const float ki, const float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  previous_error = 0.0f;
  previous_input = 0.0f;
  previous_derivative = 0.0f;
  i_term = 0.0f;
  // No raw windup clamp by default: the I-term is kept inside the output
  // range instead, which does not silently shrink when ki is small.
  min_windup = 0.0f;
  max_windup = 0.0f;
  windup_enabled = false;
  min_integral_out = 0.0f;
  max_integral_out = 0.0f;
  integral_limit_enabled = false;
  setConstrain(-255.0f, 255.0f); // after the limit flags: it re-applies them
  conditional_integration = true;
  derivative_smoothing = 0.0f; // no smoothing by default (0=no smoothing, 1=full smoothing)
  derivative_tau = 0.0f;
  use_tau_filter = false;
  dampingFactor = 1.0f;
  last_p = 0.0f;
  last_i = 0.0f;
  last_d = 0.0f;
  last_output = 0.0f;
  last_dt = 0.0f;
  last_resumed = false;
  hasPreviousInput = false;
  previous_source = SOURCE_ERROR;
  continuous_range = 0.0f;
  continuous_enabled = false;
  continuous_inv_range = 0.0f;
  error_tolerance = 0.0f;
  rate_tolerance = -1.0f;
  settle_s = 0.0f;
  tolerance_enabled = false;
  in_tolerance = false;
  settled_time = 0.0f;
  kf = 0.0f;
  ks = 0.0f;
  last_f = 0.0f;
  min_output = 0.0f;
  ramp_rate = 0.0f;
  lastMicros = 0;
  hasLastMicros = false;
  max_dt_ms = 500; // gaps longer than this are treated as a resume
}

// Internal helper: constrain float between min and max
static float constrainFloat(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Internal helper: false for NaN and +/-infinity, i.e. an IEEE 754 float
// whose exponent bits are all ones. An integer test is much cheaper than
// float arithmetic on boards without an FPU, and avoids relying on
// isfinite(), which is a macro on some cores and a std:: function on others.
static bool isFiniteFloat(float x) {
  uint32_t bits;
  memcpy(&bits, &x, sizeof bits);
  return (bits & 0x7F800000UL) != 0x7F800000UL;
}

// Clamp the I-term. It never needs to move the output by more than the
// width of the output range, so that is always enforced. The bound is
// symmetric so that, with a one-sided range such as (0, 255), the I-term can
// still go negative to trim a feedforward that overshoots. The raw windup
// clamp (scaled by ki into output units) and the output-unit integral limit
// apply on top when enabled.
void PIDEasy::applyIntegralLimits() {
  const float width = max_constrain - min_constrain;
  i_term = constrainFloat(i_term, -width, width);
  if (windup_enabled && ki != 0.0f) {
    float lo = ki * min_windup;
    float hi = ki * max_windup;
    if (lo > hi) { const float tmp = lo; lo = hi; hi = tmp; } // negative ki
    i_term = constrainFloat(i_term, lo, hi);
  }
  if (integral_limit_enabled) {
    i_term = constrainFloat(i_term, min_integral_out, max_integral_out);
  }
}

// Wrap x into [-range/2, range/2). floorf() rather than a loop so a heading
// that has accumulated many turns still wraps in constant time.
float PIDEasy::wrap(float x) {
  return x - continuous_range * floorf(x * continuous_inv_range + 0.5f);
}

// Shared implementation for every compute / update variant. dt is in seconds.
float PIDEasy::step(float error, float dInput, DerivativeSource source,
                float dt, bool resume, float feedforward) {
  // A non-finite input (e.g. a line position computed as 0/0 when every
  // sensor reads white) would otherwise latch NaN into the integral and
  // disable the controller until reset(). Hold the last output instead.
  // The update*() callers have already checked setpoint and measurement,
  // which dInput and the feedforward are derived from.
  if (!isFiniteFloat(error)) return last_output;

  if (continuous_enabled) error = wrap(error);

  const bool signChanged = (error > 0 && previous_error < 0) || (error < 0 && previous_error > 0);
  // I-term as it stood before this step, kept so conditional integration
  // can undo the step without also undoing the sign-change damping.
  const float i_before = i_term;
  // Direction this step moves the I-term, for conditional integration.
  const float ki_error = ki * error;

  // After a gap longer than max_dt_ms the elapsed error is not a meaningful
  // area, so skip the accumulation instead of adding one huge step.
  // Accumulating ki * error * dt (rather than error * dt, scaled later)
  // means a ki change affects only future error, never the stored sum.
  if (!resume) {
    i_term += ki_error * dt;
    applyIntegralLimits();
  }

  if (signChanged) {
      i_term *= dampingFactor;
  }

  // On the first sample there is no valid previous input, so the raw
  // derivative would be a spurious spike. Use 0 until we have history.
  // A resume, or a switch between compute() and update(), invalidates
  // that history in the same way.
  // The derivative costs a float division, so skip it when nothing reads
  // it: kd == 0 and no rate tolerance (e.g. a PI speed loop).
  const bool derivative_used = (kd != 0.0f) || (tolerance_enabled && rate_tolerance >= 0.0f);
  float derivative;
  if (resume || !hasPreviousInput || previous_source != source || !derivative_used) {
    derivative = 0.0f;
  } else {
    float delta = dInput - previous_input;
    // A heading crossing 180 -> -180 moved 2 degrees, not 358.
    if (continuous_enabled) delta = wrap(delta);
    derivative = delta / dt;
    // In tau mode the coefficient is rebuilt from dt each call, so the
    // filter keeps the same time constant when the loop period jitters.
    const float smoothing = use_tau_filter
        ? (derivative_tau / (derivative_tau + dt))
        : derivative_smoothing;
    // Skipped when off (the default): float math is slow without an FPU.
    if (smoothing != 0.0f) {
      derivative = derivative + smoothing * (previous_derivative - derivative);
    }
  }

  const float p_term = kp * error;
  const float d_term = kd * derivative;
  const float pdf = p_term + d_term + feedforward;
  float output = pdf + i_term;

  // Conditional integration: if the output is already past a constrain limit
  // and this step pushed it further out, roll the step back. The sign of
  // ki * error gives the direction the step moved the I-term, so this stays
  // correct whatever the sign of ki.
  if (!resume && conditional_integration) {
    const bool pushingUp = (output > max_constrain) && (ki_error > 0.0f);
    const bool pushingDown = (output < min_constrain) && (ki_error < 0.0f);
    if (pushingUp || pushingDown) {
      i_term = signChanged ? (i_before * dampingFactor) : i_before;
      output = pdf + i_term;
    }
  }

  previous_error = error;
  previous_input = dInput;
  previous_derivative = derivative;
  previous_source = source;
  hasPreviousInput = true;

  last_p = p_term;
  last_i = i_term;
  last_d = d_term;
  last_f = feedforward;

  // Settle tracking for atSetpoint(). A resume sample has no meaningful
  // elapsed time, so it neither adds to nor breaks the settle time.
  const bool error_inside = tolerance_enabled && fabsf(error) <= error_tolerance;
  if (tolerance_enabled) {
    const bool inside = error_inside &&
        (rate_tolerance < 0.0f || fabsf(derivative) <= rate_tolerance);
    if (!inside) {
      settled_time = 0.0f;
    } else if (!resume) {
      settled_time += dt;
    }
    in_tolerance = inside;
  }

  // Deadband compensation: lift small outputs to the level where the motor
  // actually moves, but not inside the tolerance band, where doing so would
  // make the robot hunt back and forth across the target.
  if (min_output > 0.0f && output != 0.0f && !error_inside &&
      fabsf(output) < min_output) {
    output = (output > 0.0f) ? min_output : -min_output;
  }

  // Slew-rate limit relative to the previous returned output.
  if (ramp_rate > 0.0f) {
    const float max_change = ramp_rate * dt;
    output = constrainFloat(output, last_output - max_change, last_output + max_change);
  }

  last_output = constrainFloat(output, min_constrain, max_constrain);
  last_dt = dt;
  last_resumed = resume;

  return last_output;
}

// Measure the time since the previous internally timed call with micros().
// Timestamps are kept as 32-bit values, as micros() is on every Arduino core,
// so the unsigned subtraction wraps correctly across the ~71 minute rollover
// even where unsigned long is 64 bits wide.
bool PIDEasy::measureTime(float* dt, bool* resume) {
  const uint32_t now = (uint32_t)micros();
  if (!hasLastMicros) {
    // No time has elapsed yet, so there is nothing to integrate or
    // differentiate: treat the first sample as a resume.
    hasLastMicros = true;
    lastMicros = now;
    *dt = 0.0f;
    *resume = true;
    return true;
  }

  const uint32_t elapsed_us = now - lastMicros;
  // Called again before the clock ticked: no measurable time has passed.
  if (elapsed_us == 0) return false;
  lastMicros = now;

  // A pause longer than the cap (e.g. robot stopped to signal a victim)
  // makes the integral and derivative for this sample meaningless, so
  // treat it as a resume rather than accumulating one giant step.
  const uint32_t cap_us = (max_dt_ms > 4294967UL) ? 0xFFFFFFFFUL
                                                  : (uint32_t)(max_dt_ms * 1000UL);
  if (max_dt_ms > 0 && elapsed_us > cap_us) {
    *dt = max_dt_ms * 0.001f;
    *resume = true;
  } else {
    *dt = elapsed_us * 1e-6f;
    *resume = false;
  }
  return true;
}

// Compute with dt specified in milliseconds.
// dt_ms == 0 is treated as 1 ms, as in earlier releases.
float PIDEasy::computeMs(float error, unsigned long dt_ms) {
  const float dt = (dt_ms == 0) ? 0.001f : (dt_ms * 0.001f);
  return step(error, error, SOURCE_ERROR, dt, false, 0.0f);
}

// Compute with dt specified in microseconds. dt_us == 0 holds.
float PIDEasy::computeUs(float error, unsigned long dt_us) {
  if (dt_us == 0) return last_output;
  return step(error, error, SOURCE_ERROR, dt_us * 1e-6f, false, 0.0f);
}

// Backwards-compatible compute: dt provided in seconds (original behavior).
float PIDEasy::compute(const float error, const unsigned long dt) {
  // Guard: if dt_seconds is zero, use 1 second as original library did.
  const unsigned long dt_sec_nonzero = (dt == 0) ? 1 : dt;
  // Convert seconds to milliseconds and call computeMs
  const unsigned long dt_ms = dt_sec_nonzero * 1000UL;
  return computeMs(error, dt_ms);
}

// Compute using micros() to determine dt. First call initializes internal timer.
float PIDEasy::compute(const float error) {
  // Checked before the timer so a rejected sample does not consume elapsed
  // time: the next valid sample then sees the true gap.
  if (!isFiniteFloat(error)) return last_output;
  float dt;
  bool resume;
  if (!measureTime(&dt, &resume)) return last_output;
  return step(error, error, SOURCE_ERROR, dt, resume, 0.0f);
}

// Compute with dt as float seconds. dt <= 0 or NaN holds.
float PIDEasy::computeSeconds(float error, float dt_s) {
  if (!(dt_s > 0.0f) || !isFiniteFloat(dt_s)) return last_output;
  return step(error, error, SOURCE_ERROR, dt_s, false, 0.0f);
}

// Setpoint / measurement variants: the derivative is taken of -measurement,
// which equals the derivative of the error whenever the setpoint is constant
// but ignores setpoint steps.
float PIDEasy::updateInternal(float setpoint, float measurement, float dt, bool resume) {
  // Guarded here too so a NaN setpoint cannot reach the feedforward.
  if (!isFiniteFloat(setpoint) || !isFiniteFloat(measurement)) return last_output;
  const float sign = (setpoint > 0.0f) ? 1.0f : ((setpoint < 0.0f) ? -1.0f : 0.0f);
  const float feedforward = kf * setpoint + ks * sign;
  return step(setpoint - measurement, -measurement, SOURCE_MEASUREMENT, dt, resume, feedforward);
}

float PIDEasy::update(const float setpoint, const float measurement) {
  if (!isFiniteFloat(setpoint) || !isFiniteFloat(measurement)) return last_output;
  float dt;
  bool resume;
  if (!measureTime(&dt, &resume)) return last_output;
  return updateInternal(setpoint, measurement, dt, resume);
}

float PIDEasy::updateMs(const float setpoint, const float measurement, const unsigned long dt_ms) {
  if (dt_ms == 0) return last_output;
  return updateInternal(setpoint, measurement, dt_ms * 0.001f, false);
}

float PIDEasy::updateUs(const float setpoint, const float measurement, const unsigned long dt_us) {
  if (dt_us == 0) return last_output;
  return updateInternal(setpoint, measurement, dt_us * 1e-6f, false);
}

float PIDEasy::updateSeconds(const float setpoint, const float measurement, const float dt_s) {
  if (!(dt_s > 0.0f) || !isFiniteFloat(dt_s)) return last_output;
  return updateInternal(setpoint, measurement, dt_s, false);
}

// Change the gains at runtime. Internal state (I-term, derivative history,
// timer) is deliberately preserved so mode switches stay bumpless.
void PIDEasy::setTunings(const float kp, const float ki, const float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  // With ki == 0 the integral is switched off, so an I-term frozen from the
  // previous mode would only be a hidden constant offset.
  if (ki == 0.0f) i_term = 0.0f;
  // The raw windup clamp is scaled by ki, so re-apply the limits.
  applyIntegralLimits();
}

float PIDEasy::getKp() const { return this->kp; }

float PIDEasy::getKi() const { return this->ki; }

float PIDEasy::getKd() const { return this->kd; }

// Set the minimum and maximum output of the PID controller.
// Arguments are swapped automatically if given in the wrong order.
void PIDEasy::setConstrain(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_constrain = min;
  this->max_constrain = max;
  // The I-term is bounded by the width of the output range.
  applyIntegralLimits();
}

// Reset the PID controller's internal state.
void PIDEasy::reset() {
  previous_error = 0.0f;
  previous_input = 0.0f;
  previous_derivative = 0.0f;
  i_term = 0.0f;
  hasPreviousInput = false;
  hasLastMicros = false;
  lastMicros = 0;
  last_p = 0.0f;
  last_i = 0.0f;
  last_d = 0.0f;
  last_output = 0.0f;
  last_f = 0.0f;
  last_dt = 0.0f;
  last_resumed = false;
  in_tolerance = false;
  settled_time = 0.0f;
}

// Clear only the I-term.
void PIDEasy::resetIntegral() {
  i_term = 0.0f;
}

// Preload the I-term in output units, within the integral limits.
void PIDEasy::setIntegral(float value) {
  if (!isFiniteFloat(value)) return;
  i_term = value;
  applyIntegralLimits();
}

// Set the smoothing factor for the derivative term to reduce noise sensitivity.
// Expect sD in [0..1]. Values near 1 use more of the previous derivative (more smoothing).
void PIDEasy::setSmoothingDerivative(float sD) {
  if (sD < 0.0f) sD = 0.0f;
  if (sD > 1.0f) sD = 1.0f;
  this->derivative_smoothing = sD;
  this->use_tau_filter = false; // fixed coefficient takes over from tau mode
}

// Backwards-compatible alias for setSmoothingDerivative().
void PIDEasy::setSmoothingDerivate(float sD) {
  setSmoothingDerivative(sD);
}

// Low-pass the derivative with a time constant in seconds. The smoothing
// coefficient becomes tau / (tau + dt), recomputed every call, so a jittery
// loop period no longer changes how much smoothing is applied.
// 0 (or negative) turns derivative filtering off entirely.
void PIDEasy::setDerivativeTimeConstant(float tauSeconds) {
  if (tauSeconds <= 0.0f) {
    this->derivative_tau = 0.0f;
    this->use_tau_filter = false;
    this->derivative_smoothing = 0.0f;
    return;
  }
  this->derivative_tau = tauSeconds;
  this->use_tau_filter = true;
}

// Clamp the raw integral (error x seconds) to [min, max]. Enabling it is
// permanent for the object; it is applied as ki * limit on the I-term.
// Arguments are swapped automatically if given in the wrong order.
void PIDEasy::setWindUP(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_windup = min;
  this->max_windup = max;
  this->windup_enabled = true;
  applyIntegralLimits();
}

// Limit the integral's contribution to the output (ki * integral) rather than
// the raw integral, so the limit does not change meaning when ki is retuned.
// Arguments are swapped automatically if given in the wrong order.
// (0, 0) disables the limit.
void PIDEasy::setIntegralLimit(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_integral_out = min;
  this->max_integral_out = max;
  this->integral_limit_enabled = !(min == 0.0f && max == 0.0f);
  applyIntegralLimits();
}

// Enable or disable conditional integration (enabled by default).
void PIDEasy::setConditionalIntegration(bool enabled) {
  this->conditional_integration = enabled;
}

// Set the damping factor for the integral term when the error changes sign.
// Clamped to [0..1]: values above 1 would amplify the integral at every
// zero crossing and values below 0 would flip its sign.
void PIDEasy::setDampingFactor(float dF) {
  if (dF < 0.0f) dF = 0.0f;
  if (dF > 1.0f) dF = 1.0f;
  this->dampingFactor = dF;
}

// Wrap the error (and derivative) into half of the given circular range.
// Arguments are swapped automatically; a zero-width range disables wrapping.
void PIDEasy::setContinuousInput(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->continuous_range = max - min;
  this->continuous_enabled = (max > min) && isFiniteFloat(continuous_range);
  this->continuous_inv_range = continuous_enabled ? 1.0f / continuous_range : 0.0f;
}

void PIDEasy::disableContinuousInput() {
  this->continuous_enabled = false;
}

// Tolerance used by atSetpoint(). A negative rateTolerance is not checked.
void PIDEasy::setTolerance(float errorTolerance, float rateTolerance, unsigned long settleMs) {
  this->error_tolerance = fabsf(errorTolerance);
  this->rate_tolerance = rateTolerance;
  this->settle_s = settleMs * 0.001f;
  this->tolerance_enabled = true;
  this->in_tolerance = false;
  this->settled_time = 0.0f;
}

bool PIDEasy::atSetpoint() const {
  return tolerance_enabled && in_tolerance && settled_time >= settle_s;
}

// Feedforward for update*(): kF * setpoint + kS * sign(setpoint).
void PIDEasy::setFeedforward(float kF, float kS) {
  this->kf = kF;
  this->ks = kS;
}

// Minimum nonzero output magnitude (motor deadband). 0 disables.
void PIDEasy::setMinOutput(float minOutput) {
  this->min_output = (minOutput > 0.0f) ? minOutput : 0.0f;
}

// Maximum output change per second. 0 disables.
void PIDEasy::setOutputRampRate(float unitsPerSecond) {
  this->ramp_rate = (unitsPerSecond > 0.0f) ? unitsPerSecond : 0.0f;
}

// Longest gap that still counts as a normal cycle for the internally timed
// variants. 0 disables the cap.
void PIDEasy::setMaxDeltaTime(unsigned long maxDtMs) {
  this->max_dt_ms = maxDtMs;
}

// Per-term contributions from the last compute call, for tuning telemetry.
float PIDEasy::getP() const { return this->last_p; }

float PIDEasy::getI() const { return this->last_i; }

float PIDEasy::getD() const { return this->last_d; }

float PIDEasy::getOutput() const { return this->last_output; }

float PIDEasy::getF() const { return this->last_f; }

// Timing diagnostics for the last sample that ran.
float PIDEasy::getDeltaTime() const { return this->last_dt; }

bool PIDEasy::wasResumed() const { return this->last_resumed; }
