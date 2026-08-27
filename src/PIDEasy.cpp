#include "PIDEasy.h"

// Constructor to initialize the PID controller.
PID::PID(const float kp, const float ki, const float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  previous_error = 0.0f;
  previous_derivative = 0.0f;
  integral = 0.0f;
  setWindUP(-255.0f, 255.0f);
  setConstrain(-255.0f, 255.0f);
  min_integral_out = 0.0f;
  max_integral_out = 0.0f;
  integral_limit_enabled = false;
  conditional_integration = true;
  derivative_smoothing = 0.0f; // no smoothing by default (0=no smoothing, 1=full smoothing)
  derivative_tau = 0.0f;
  use_tau_filter = false;
  dampingFactor = 1.0f;
  last_p = 0.0f;
  last_i = 0.0f;
  last_d = 0.0f;
  last_output = 0.0f;
  hasPreviousError = false;
  lastMillis = 0;
  hasLastMillis = false;
  max_dt_ms = 100; // cap internally measured dt at 100 ms by default
}

// Internal helper: constrain float between min and max
static float constrainFloat(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Clamp the integral to the windup limits, then (if enabled) to the limit
// expressed in output units. Dividing by ki converts that bound back into a
// bound on the raw integral; skipped when ki is not positive, where the
// conversion is meaningless or would flip the interval.
void PID::applyIntegralLimits() {
  integral = constrainFloat(integral, min_windup, max_windup);
  if (integral_limit_enabled && ki > 0.0f) {
    integral = constrainFloat(integral, min_integral_out / ki, max_integral_out / ki);
  }
}

// Shared implementation for every compute variant. dt is in milliseconds.
float PID::computeInternal(float error, unsigned long dt_ms, bool resume) {
  // Convert dt to seconds for internal calculations
  const float dt = (dt_ms == 0) ? 0.001f : (dt_ms / 1000.0f);

  const bool signChanged = (error > 0 && previous_error < 0) || (error < 0 && previous_error > 0);
  // Integral as it stood before this step, kept so conditional integration
  // can undo the step without also undoing the sign-change damping.
  const float integral_before = integral;

  // After a gap longer than max_dt_ms the elapsed error is not a meaningful
  // area, so skip the accumulation instead of adding one huge step.
  if (!resume) {
    integral += error * dt;
    applyIntegralLimits();
  }

  if (signChanged) {
      integral *= dampingFactor;
  }

  // On the first sample there is no valid previous_error, so the raw
  // derivative would be a spurious spike. Use 0 until we have history.
  // A resume invalidates that history in the same way.
  float derivative;
  if (resume || !hasPreviousError) {
    derivative = 0.0f;
  } else {
    derivative = (error - previous_error) / dt;
    // In tau mode the coefficient is rebuilt from dt each call, so the
    // filter keeps the same time constant when the loop period jitters.
    const float smoothing = use_tau_filter
        ? (derivative_tau / (derivative_tau + dt))
        : derivative_smoothing;
    derivative = (smoothing * previous_derivative) + ((1.0f - smoothing) * derivative);
  }

  float output = (kp * error) + (ki * integral) + (kd * derivative);

  // Conditional integration: if the output is already past a constrain limit
  // and this step pushed it further out, roll the step back. The sign of
  // ki * error gives the direction the step moved the I-term, so this stays
  // correct whatever the sign of ki.
  if (!resume && conditional_integration) {
    const bool pushingUp = (output > max_constrain) && (ki * error > 0.0f);
    const bool pushingDown = (output < min_constrain) && (ki * error < 0.0f);
    if (pushingUp || pushingDown) {
      integral = signChanged ? (integral_before * dampingFactor) : integral_before;
      output = (kp * error) + (ki * integral) + (kd * derivative);
    }
  }

  previous_error = error;
  previous_derivative = derivative;
  hasPreviousError = true;

  last_p = kp * error;
  last_i = ki * integral;
  last_d = kd * derivative;
  last_output = constrainFloat(output, min_constrain, max_constrain);

  return last_output;
}

// Compute with dt specified in milliseconds.
float PID::computeMs(float error, unsigned long dt_ms) {
  return computeInternal(error, dt_ms, false);
}

// Backwards-compatible compute: dt provided in seconds (original behavior).
float PID::compute(const float error, const unsigned long dt) {
  // Guard: if dt_seconds is zero, use 1 second as original library did.
  const unsigned long dt_sec_nonzero = (dt == 0) ? 1 : dt;
  // Convert seconds to milliseconds and call computeMs
  const unsigned long dt_ms = dt_sec_nonzero * 1000UL;
  return computeMs(error, dt_ms);
}

// Compute using millis() to determine dt. First call initializes internal timer.
float PID::compute(const float error) {
  const unsigned long now = millis();
  unsigned long dt_ms = 0;
  bool resume = false;
  if (!hasLastMillis) {
    hasLastMillis = true;
    lastMillis = now;
    dt_ms = 1; // small non-zero dt
  } else {
    dt_ms = now - lastMillis;
    lastMillis = now;
    if (dt_ms == 0) dt_ms = 1;
    // A pause longer than the cap (e.g. robot stopped to signal a victim)
    // makes the integral and derivative for this sample meaningless, so
    // treat it as a resume rather than accumulating one giant step.
    if (max_dt_ms > 0 && dt_ms > max_dt_ms) {
      dt_ms = max_dt_ms;
      resume = true;
    }
  }
  return computeInternal(error, dt_ms, resume);
}

// Change the gains at runtime. Internal state (integral, derivative history,
// millis() timer) is deliberately preserved so mode switches stay bumpless.
void PID::setTunings(const float kp, const float ki, const float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  // The output-unit integral limit is relative to ki, so re-apply it.
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
}

// Reset the PID controller's internal state.
void PID::reset() {
  previous_error = 0.0f;
  previous_derivative = 0.0f;
  integral = 0.0f;
  hasPreviousError = false;
  hasLastMillis = false;
  lastMillis = 0;
  last_p = 0.0f;
  last_i = 0.0f;
  last_d = 0.0f;
  last_output = 0.0f;
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

// Set the windup limits for the integral term to prevent integral windup.
// Arguments are swapped automatically if given in the wrong order.
void PID::setWindUP(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_windup = min;
  this->max_windup = max;
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

// Cap the dt measured internally by compute(error). 0 disables the cap.
void PID::setMaxDeltaTime(unsigned long maxDtMs) {
  this->max_dt_ms = maxDtMs;
}

// Per-term contributions from the last compute call, for tuning telemetry.
float PID::getP() { return this->last_p; }

float PID::getI() { return this->last_i; }

float PID::getD() { return this->last_d; }

float PID::getOutput() { return this->last_output; }
