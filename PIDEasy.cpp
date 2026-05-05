#include "PIDEasy.h"
#include <Arduino.h> // Necessary for the constrain function and other Arduino-specific features

// Constructor to initialize the PID controller.
PID::PID(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  previous_error = 0;
  previous_derivative = 0;
  integral = 0;
  setWindUP(-255, 255);
  setConstrain(-255, 255);
  derivative_smoothing = 1.0;
  dampingFactor = 1.0;
}

// Compute the PID output.
float PID::compute(float error, unsigned long dt) {
  if (dt == 0) dt = 1;  

  integral += error * dt;
  integral = constrain(integral, min_windup, max_windup);

  if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0)) {
      integral *= dampingFactor;
  }

  float derivative = (error - previous_error) / dt;
  derivative = (derivative_smoothing * previous_derivative) + ((1 - derivative_smoothing) * derivative);

  float output = (kp * error) + (ki * integral) + (kd * derivative);

  previous_error = error;
  previous_derivative = derivative;

  return constrain(output, min_constrain, max_constrain);
}

// Set the minimum and maximum output of the PID controller.
void PID::setConstrain(int min, int max) {
  this->min_constrain = min;
  this->max_constrain = max;
}

// Reset the PID controller's internal state.
void PID::reset() {
  previous_error = 0;
  previous_derivative = 0;
  integral = 0;
}

// Set the smoothing factor for the derivative term to reduce noise sensitivity.
void PID::setSmoothingDerivate(float sD) {
  this->derivative_smoothing = sD;
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
