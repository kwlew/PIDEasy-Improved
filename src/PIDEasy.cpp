#include "PIDEasy.h"

// Constructor to initialize the PID controller.
PID::PID(const float kp, const float ki, const float kd) {
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

// Internal helper: false for NaN and +/-infinity. NaN fails x == x, and an
// infinity fails x - x == 0 (inf - inf is NaN). Avoids relying on isfinite(),
// which is a macro on some cores and a std:: function on others.
static bool isFiniteFloat(float x) {
  return (x == x) && (x - x == 0.0f);
}

// Clamp the I-term. It never needs to exceed the output range on its own,
// so that is always enforced; the raw windup clamp (scaled by ki into output
// units) and the output-unit integral limit apply on top when enabled.
void PID::applyIntegralLimits() {
  i_term = constrainFloat(i_term, min_constrain, max_constrain);
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

// Shared implementation for every compute / update variant. dt is in seconds.
float PID::step(float error, float dInput, DerivativeSource source,
                float dt, bool resume) {
  // A non-finite input (e.g. a line position computed as 0/0 when every
  // sensor reads white) would otherwise latch NaN into the integral and
  // disable the controller until reset(). Hold the last output instead.
  if (!isFiniteFloat(error) || !isFiniteFloat(dInput)) return last_output;

  const bool signChanged = (error > 0 && previous_error < 0) || (error < 0 && previous_error > 0);
  // I-term as it stood before this step, kept so conditional integration
  // can undo the step without also undoing the sign-change damping.
  const float i_before = i_term;

  // After a gap longer than max_dt_ms the elapsed error is not a meaningful
  // area, so skip the accumulation instead of adding one huge step.
  // Accumulating ki * error * dt (rather than error * dt, scaled later)
  // means a ki change affects only future error, never the stored sum.
  if (!resume) {
    i_term += ki * error * dt;
    applyIntegralLimits();
  }

  if (signChanged) {
      i_term *= dampingFactor;
  }

  // On the first sample there is no valid previous input, so the raw
  // derivative would be a spurious spike. Use 0 until we have history.
  // A resume, or a switch between compute() and update(), invalidates
  // that history in the same way.
  float derivative;
  if (resume || !hasPreviousInput || previous_source != source) {
    derivative = 0.0f;
  } else {
    derivative = (dInput - previous_input) / dt;
    // In tau mode the coefficient is rebuilt from dt each call, so the
    // filter keeps the same time constant when the loop period jitters.
    const float smoothing = use_tau_filter
        ? (derivative_tau / (derivative_tau + dt))
        : derivative_smoothing;
    derivative = (smoothing * previous_derivative) + ((1.0f - smoothing) * derivative);
  }

  float output = (kp * error) + i_term + (kd * derivative);

  // Conditional integration: if the output is already past a constrain limit
  // and this step pushed it further out, roll the step back. The sign of
  // ki * error gives the direction the step moved the I-term, so this stays
  // correct whatever the sign of ki.
  if (!resume && conditional_integration) {
    const bool pushingUp = (output > max_constrain) && (ki * error > 0.0f);
    const bool pushingDown = (output < min_constrain) && (ki * error < 0.0f);
    if (pushingUp || pushingDown) {
      i_term = signChanged ? (i_before * dampingFactor) : i_before;
      output = (kp * error) + i_term + (kd * derivative);
    }
  }

  previous_error = error;
  previous_input = dInput;
  previous_derivative = derivative;
  previous_source = source;
  hasPreviousInput = true;

  last_p = kp * error;
  last_i = i_term;
  last_d = kd * derivative;
  last_output = constrainFloat(output, min_constrain, max_constrain);
  last_dt = dt;
  last_resumed = resume;

  return last_output;
}

// Measure the time since the previous internally timed call with micros().
// Timestamps are kept as 32-bit values, as micros() is on every Arduino core,
// so the unsigned subtraction wraps correctly across the ~71 minute rollover
// even where unsigned long is 64 bits wide.
bool PID::measureTime(float* dt, bool* resume) {
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
float PID::computeMs(float error, unsigned long dt_ms) {
  const float dt = (dt_ms == 0) ? 0.001f : (dt_ms * 0.001f);
  return step(error, error, SOURCE_ERROR, dt, false);
}

// Compute with dt specified in microseconds. dt_us == 0 holds.
float PID::computeUs(float error, unsigned long dt_us) {
  if (dt_us == 0) return last_output;
  return step(error, error, SOURCE_ERROR, dt_us * 1e-6f, false);
}

// Backwards-compatible compute: dt provided in seconds (original behavior).
float PID::compute(const float error, const unsigned long dt) {
  // Guard: if dt_seconds is zero, use 1 second as original library did.
  const unsigned long dt_sec_nonzero = (dt == 0) ? 1 : dt;
  // Convert seconds to milliseconds and call computeMs
  const unsigned long dt_ms = dt_sec_nonzero * 1000UL;
  return computeMs(error, dt_ms);
}

// Compute using micros() to determine dt. First call initializes internal timer.
float PID::compute(const float error) {
  // Checked before the timer so a rejected sample does not consume elapsed
  // time: the next valid sample then sees the true gap.
  if (!isFiniteFloat(error)) return last_output;
  float dt;
  bool resume;
  if (!measureTime(&dt, &resume)) return last_output;
  return step(error, error, SOURCE_ERROR, dt, resume);
}

// Setpoint / measurement variants: the derivative is taken of -measurement,
// which equals the derivative of the error whenever the setpoint is constant
// but ignores setpoint steps.
float PID::update(const float setpoint, const float measurement) {
  if (!isFiniteFloat(setpoint) || !isFiniteFloat(measurement)) return last_output;
  float dt;
  bool resume;
  if (!measureTime(&dt, &resume)) return last_output;
  return step(setpoint - measurement, -measurement, SOURCE_MEASUREMENT, dt, resume);
}

float PID::updateMs(const float setpoint, const float measurement, const unsigned long dt_ms) {
  if (dt_ms == 0) return last_output;
  return step(setpoint - measurement, -measurement, SOURCE_MEASUREMENT, dt_ms * 0.001f, false);
}

float PID::updateUs(const float setpoint, const float measurement, const unsigned long dt_us) {
  if (dt_us == 0) return last_output;
  return step(setpoint - measurement, -measurement, SOURCE_MEASUREMENT, dt_us * 1e-6f, false);
}

// Change the gains at runtime. Internal state (I-term, derivative history,
// timer) is deliberately preserved so mode switches stay bumpless.
void PID::setTunings(const float kp, const float ki, const float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  // With ki == 0 the integral is switched off, so an I-term frozen from the
  // previous mode would only be a hidden constant offset.
  if (ki == 0.0f) i_term = 0.0f;
  // The raw windup clamp is scaled by ki, so re-apply the limits.
  applyIntegralLimits();
}

float PID::getKp() { return this->kp; }

float PID::getKi() { return this->ki; }

float PID::getKd() { return this->kd; }

// Set the minimum and maximum output of the PID controller.
// Arguments are swapped automatically if given in the wrong order.
void PID::setConstrain(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_constrain = min;
  this->max_constrain = max;
  // The I-term is bounded by the output range.
  applyIntegralLimits();
}

// Reset the PID controller's internal state.
void PID::reset() {
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
  last_dt = 0.0f;
  last_resumed = false;
}

// Set the smoothing factor for the derivative term to reduce noise sensitivity.
// Expect sD in [0..1]. Values near 1 use more of the previous derivative (more smoothing).
void PID::setSmoothingDerivative(float sD) {
  if (sD < 0.0f) sD = 0.0f;
  if (sD > 1.0f) sD = 1.0f;
  this->derivative_smoothing = sD;
  this->use_tau_filter = false; // fixed coefficient takes over from tau mode
}

// Backwards-compatible alias for setSmoothingDerivative().
void PID::setSmoothingDerivate(float sD) {
  setSmoothingDerivative(sD);
}

// Low-pass the derivative with a time constant in seconds. The smoothing
// coefficient becomes tau / (tau + dt), recomputed every call, so a jittery
// loop period no longer changes how much smoothing is applied.
// 0 (or negative) turns derivative filtering off entirely.
void PID::setDerivativeTimeConstant(float tauSeconds) {
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
void PID::setWindUP(float min, float max) {
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
void PID::setIntegralLimit(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_integral_out = min;
  this->max_integral_out = max;
  this->integral_limit_enabled = !(min == 0.0f && max == 0.0f);
  applyIntegralLimits();
}

// Enable or disable conditional integration (enabled by default).
void PID::setConditionalIntegration(bool enabled) {
  this->conditional_integration = enabled;
}

// Set the damping factor for the integral term when the error changes sign.
// Clamped to [0..1]: values above 1 would amplify the integral at every
// zero crossing and values below 0 would flip its sign.
void PID::setDampingFactor(float dF) {
  if (dF < 0.0f) dF = 0.0f;
  if (dF > 1.0f) dF = 1.0f;
  this->dampingFactor = dF;
}

// Longest gap that still counts as a normal cycle for the internally timed
// variants. 0 disables the cap.
void PID::setMaxDeltaTime(unsigned long maxDtMs) {
  this->max_dt_ms = maxDtMs;
}

// Per-term contributions from the last compute call, for tuning telemetry.
float PID::getP() { return this->last_p; }

float PID::getI() { return this->last_i; }

float PID::getD() { return this->last_d; }

float PID::getOutput() { return this->last_output; }

// Timing diagnostics for the last sample that ran.
float PID::getDeltaTime() { return this->last_dt; }

bool PID::wasResumed() { return this->last_resumed; }
