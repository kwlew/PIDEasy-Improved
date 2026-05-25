#include "PIDEasy.h"

// Constructor to initialize the PID controller.
PID::PID(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  previous_error = 0.0f;
  previous_derivative = 0.0f;
  integral = 0.0f;
  setWindUP(-255.0f, 255.0f);
  setConstrain(-255.0f, 255.0f);
  derivative_smoothing = 1.0f; // no smoothing by default
  dampingFactor = 1.0f;
  lastMillis = 0;
  hasLastMillis = false;
}

// Internal helper: constrain float between min and max
static float constrainFloat(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Compute the PID output. dt is in milliseconds.
float PID::compute(float error, unsigned long dt_ms) {
  // Convert dt to seconds for consistent units
  float dt = (dt_ms == 0) ? 0.001f : (dt_ms / 1000.0f);

  integral += error * dt;
  integral = constrainFloat(integral, min_windup, max_windup);

  if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0)) {
      integral *= dampingFactor;
  }

  float derivative = (error - previous_error) / dt;
  derivative = (derivative_smoothing * previous_derivative) + ((1.0f - derivative_smoothing) * derivative);

  float output = (kp * error) + (ki * integral) + (kd * derivative);

  previous_error = error;
  previous_derivative = derivative;

  return constrainFloat(output, min_constrain, max_constrain);
}

// Compute using millis() to determine dt. First call initializes internal timer.
float PID::compute(float error) {
  unsigned long now = millis();
  unsigned long dt_ms = 0;
  if (!hasLastMillis) {
    hasLastMillis = true;
    lastMillis = now;
    dt_ms = 1; // small non-zero dt
  } else {
    dt_ms = now - lastMillis;
    lastMillis = now;
    if (dt_ms == 0) dt_ms = 1;
  }
  return compute(error, dt_ms);
}

// Set the minimum and maximum output of the PID controller.
void PID::setConstrain(float min, float max) {
  this->min_constrain = min;
  this->max_constrain = max;
}

// Reset the PID controller's internal state.
void PID::reset() {
  previous_error = 0.0f;
  previous_derivative = 0.0f;
  integral = 0.0f;
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

// Backwards-compatible alias for the original misspelled API.
void PID::setSmoothingDerivate(float sD) {
  setSmoothingDerivative(sD);
}

// Set the windup limits for the integral term to prevent integral windup.
void PID::setWindUP(float min, float max) {
  this->min_windup = min;
  this->max_windup = max;
}

// Set the damping factor for the integral term when the error changes sign.
void PID::setDampingFactor(float dF) {
  this->dampingFactor = dF;
}
