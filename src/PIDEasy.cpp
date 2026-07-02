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
  derivative_smoothing = 0.0f; // no smoothing by default (0=no smoothing, 1=full smoothing)
  dampingFactor = 1.0f;
  hasPreviousError = false;
  lastMillis = 0;
  hasLastMillis = false;
  max_dt_ms = 1000; // cap internally measured dt at 1 s by default
}

// Internal helper: constrain float between min and max
static float constrainFloat(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Compute the PID output. dt is in milliseconds.
// Compute with dt specified in milliseconds.
float PID::computeMs(float error, unsigned long dt_ms) {
  // Convert dt to seconds for internal calculations
  const float dt = (dt_ms == 0) ? 0.001f : (dt_ms / 1000.0f);

  integral += error * dt;
  integral = constrainFloat(integral, min_windup, max_windup);

  if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0)) {
      integral *= dampingFactor;
  }

  // On the first sample there is no valid previous_error, so the raw
  // derivative would be a spurious spike. Use 0 until we have history.
  float derivative = hasPreviousError ? ((error - previous_error) / dt) : 0.0f;
  derivative = (derivative_smoothing * previous_derivative) + ((1.0f - derivative_smoothing) * derivative);

  const float output = (kp * error) + (ki * integral) + (kd * derivative);

  previous_error = error;
  previous_derivative = derivative;
  hasPreviousError = true;

  return constrainFloat(output, min_constrain, max_constrain);
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
  if (!hasLastMillis) {
    hasLastMillis = true;
    lastMillis = now;
    dt_ms = 1; // small non-zero dt
  } else {
    dt_ms = now - lastMillis;
    lastMillis = now;
    if (dt_ms == 0) dt_ms = 1;
    // Cap dt so a long pause (e.g. robot stopped to signal a victim)
    // doesn't produce one giant integral step on resume.
    if (max_dt_ms > 0 && dt_ms > max_dt_ms) dt_ms = max_dt_ms;
  }
  return computeMs(error, dt_ms);
}

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
}

// Set the smoothing factor for the derivative term to reduce noise sensitivity.
// Expect sD in [0..1]. Values near 1 use more of the previous derivative (more smoothing).
void PID::setSmoothingDerivative(float sD) {
  if (sD < 0.0f) sD = 0.0f;
  if (sD > 1.0f) sD = 1.0f;
  this->derivative_smoothing = sD;
}

// Backwards-compatible alias for setSmoothingDerivative().
void PID::setSmoothingDerivate(float sD) {
  setSmoothingDerivative(sD);
}

// Set the windup limits for the integral term to prevent integral windup.
// Arguments are swapped automatically if given in the wrong order.
void PID::setWindUP(float min, float max) {
  if (min > max) { const float tmp = min; min = max; max = tmp; }
  this->min_windup = min;
  this->max_windup = max;
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
