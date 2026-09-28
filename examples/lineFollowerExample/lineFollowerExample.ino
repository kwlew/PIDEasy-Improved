// Line follower: analog reflectance sensors + two motors.
//
// The sensors give a line position; the PID turns that into a steering
// correction added to one motor and subtracted from the other.
//
// Wiring (edit the pins below): a row of analog reflectance sensors, left to
// right, and a motor driver with one PWM and one direction pin per motor
// (TB6612FNG, L298N, DRV8833 in PWM/DIR mode, ...).

#include <PIDEasy.h>

#if defined(ARDUINO_ARCH_ESP32)
const uint8_t SENSOR_PINS[] = {36, 39, 34, 35};  // ADC1 pins
const uint8_t LEFT_PWM = 25, LEFT_DIR = 26, RIGHT_PWM = 27, RIGHT_DIR = 14;
#else
const uint8_t SENSOR_PINS[] = {A0, A1, A2, A3};
const uint8_t LEFT_PWM = 5, LEFT_DIR = 4, RIGHT_PWM = 6, RIGHT_DIR = 7;
#endif
const uint8_t SENSOR_COUNT = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

// Readings below this count as "no line under this sensor".
const int LINE_THRESHOLD = 200;
const int BASE_SPEED = 150;  // PWM when driving straight

// Kp, Ki, Kd. Error is in sensor spacings (-1.5 .. +1.5 for 4 sensors),
// output is a PWM difference between the motors.
PIDEasy steering(90.0, 0.0, 4.0);

// Weighted average of the sensors: 0 when the line is centered, negative
// when it is to the left. If no sensor sees the line this is 0 / 0 = NaN,
// which the PID rejects (it holds its last output) instead of breaking.
float linePosition() {
  float weighted = 0.0f, total = 0.0f;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    int value = analogRead(SENSOR_PINS[i]);
    if (value < LINE_THRESHOLD) value = 0;
    weighted += value * (i - (SENSOR_COUNT - 1) / 2.0f);
    total += value;
  }
  return weighted / total;
}

void drive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(LEFT_DIR, left >= 0 ? HIGH : LOW);
  digitalWrite(RIGHT_DIR, right >= 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM, abs(left));
  analogWrite(RIGHT_PWM, abs(right));
}

void setup() {
  Serial.begin(115200);  // fast baud: printing at 9600 stalls the loop
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // The correction may use the full speed range in either direction.
  steering.setConstrain(-BASE_SPEED - 100, BASE_SPEED + 100);
  // Low-pass the D-term with a time constant (seconds), which works the
  // same whatever the loop rate is.
  steering.setDerivativeTimeConstant(0.01);
}

unsigned long lastLineSeen = 0;
unsigned long lastPrint = 0;

void loop() {
  const float position = linePosition();
  const bool lineVisible = position == position;  // false for NaN

  // Setpoint 0 (centered). update() measures dt itself with micros().
  const float correction = steering.update(0.0, position);

  if (lineVisible) {
    lastLineSeen = millis();
    drive(BASE_SPEED - correction, BASE_SPEED + correction);
  } else if (millis() - lastLineSeen > 300) {
    // Lost the line for a while: the PID is holding its last correction,
    // so stop and let the robot's own logic (gap, search) take over.
    drive(0, 0);
  }

  // Print a few times a second only; printing every loop slows it down.
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(position);             Serial.print('\t');
    Serial.print(steering.getP());      Serial.print('\t');
    Serial.print(steering.getD());      Serial.print('\t');
    Serial.println(steering.getOutput());
  }
}
