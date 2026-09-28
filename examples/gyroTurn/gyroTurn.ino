// Turn in place to a gyro heading, and know when the turn is finished.
//
// Shows the helpers most turn functions end up writing by hand:
//  - setContinuousInput(): 350 -> 10 degrees turns 20 degrees, not 340
//  - update(target, heading): a new target does not kick the D-term
//  - setTolerance() + atSetpoint(): "finished" = close, slow, and settled
//  - setMinOutput(): enough power to actually move near the end of a turn
//
// readHeading() is a placeholder: replace it with your IMU (MPU6050, BNO055,
// BNO085, ...). It must return degrees, increasing clockwise.

#include <PIDEasy.h>

#if defined(ARDUINO_ARCH_ESP32)
const uint8_t LEFT_PWM = 25, LEFT_DIR = 26, RIGHT_PWM = 27, RIGHT_DIR = 14;
#else
const uint8_t LEFT_PWM = 5, LEFT_DIR = 4, RIGHT_PWM = 6, RIGHT_DIR = 7;
#endif

PIDEasy turnPID(4.0, 0.0, 0.25);

float readHeading() {
  // Placeholder so the sketch compiles: replace with your IMU reading.
  return 0.0;
}

void drive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(LEFT_DIR, left >= 0 ? HIGH : LOW);
  digitalWrite(RIGHT_DIR, right >= 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM, abs(left));
  analogWrite(RIGHT_PWM, abs(right));
}

// Turn to an absolute heading. Returns false if it timed out (robot stuck
// against a wall, IMU not responding, gains too low).
bool turnTo(float targetDegrees, unsigned long timeoutMs) {
  turnPID.reset();  // fresh integral, derivative history and timer
  const unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    const float power = turnPID.update(targetDegrees, readHeading());
    drive(power, -power);
    if (turnPID.atSetpoint()) {
      drive(0, 0);
      return true;
    }
  }
  drive(0, 0);
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  turnPID.setConstrain(-200, 200);
  turnPID.setContinuousInput(-180, 180);
  // Finished when within 2 degrees, rotating slower than 15 deg/s, for 100 ms.
  turnPID.setTolerance(2.0, 15.0, 100);
  // Below ~40 PWM the wheels do not turn at all; lift small outputs to 40,
  // except inside the 2 degree band so the robot does not hunt.
  turnPID.setMinOutput(40);
  turnPID.setDerivativeTimeConstant(0.02);
}

void loop() {
  const float targets[] = {90, 180, -90, 0};
  for (uint8_t i = 0; i < 4; i++) {
    const bool ok = turnTo(targets[i], 3000);
    Serial.print("turn to ");
    Serial.print(targets[i]);
    Serial.println(ok ? " done" : " TIMED OUT");
    delay(500);
  }
}
