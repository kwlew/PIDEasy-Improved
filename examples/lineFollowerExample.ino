#include "PIDEasy.h"


// First argument: Kp (proportional gain)
// Second argument: Ki (integral gain)
// Third argument: Kd (derivative gain)
PID lineFollower(1.0, 0.0, 0.0);

int error = 0;

// The desired sensor value when perfectly centered.
const int setPoint = 10;

void setup() {
  Serial.begin(9600);
  lineFollower.setConstrain(-100.0, 100.0);
  lineFollower.setWindUP(-100.0, 100.0);
  lineFollower.setDampingFactor(0.5);
  lineFollower.setSmoothingDerivate(0.5); // alias to the correctly spelled setter
}

void loop() {
  int measurement = analogRead(A0);
  error = measurement - setPoint;

  // Use the overload that uses `millis()` internally to compute dt (in milliseconds).
  float output = lineFollower.compute(error);

  Serial.println(String("Output: ") + String(output));
  delay(50);
}
