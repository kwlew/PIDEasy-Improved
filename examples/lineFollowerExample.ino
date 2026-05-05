#include "PIDEasy.h"


// First argument: Kp (proportional gain)
// Second argument: Ki (integral gain)
// Third argument: Kd (derivative gain)
PID lineFollower(1.0, 0.0, 0.0);

int error = 0;

// setPoint stands as the goal. For example:
// In a line following robot, you want to center with the line.
// The setPoint is the value the sensor reads when the robot is perfectly centered, as that is what we aim to achieve.
const int setPoint = 10;

void setup() {
  Serial.begin(9600);
  lineFollower.setConstrain(-100, 100); // Sets the minimum and maximum output of the PID controller.
  // Defaults to -255 and 255, which is suitable for controlling motor speed with PWM signals. Adjust as needed for your application.
  lineFollower.setWindUP(-100, 100); // Sets the minimum and maximum integral term to prevent integral windup.
  // Defaults to -255 and 255.
  lineFollower.setDampingFactor(0.5); // Sets the damping factor for the integral term when the error changes sign.
  // Defaults to 1.0 (no damping).
  lineFollower.setSmoothingDerivate(0.5); // Sets the smoothing factor for the derivative term to reduce noise sensitivity.
  // Defaults to 1.0 (no smoothing).
}

void loop() {
  int measurement = analogRead(A0); // Read the sensor value.
  error = measurement - setPoint;
  // Calculate the PID output based on the information available to us.
  float output = lineFollower.compute(error, 1); // (error, dt), dt is the time in seconds since the last call to compute. In this example, we assume a fixed time step of 1 second for simplicity.

  Serial.println("Output: " + String(output));
}
