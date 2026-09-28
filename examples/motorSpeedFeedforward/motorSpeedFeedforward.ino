// Hold a wheel at a target speed with an encoder, feedforward and a PI loop.
//
// Feedforward does most of the work: kF * target is roughly the PWM the
// motor needs for that speed, and kS is the PWM needed just to overcome
// friction. The PI loop only corrects what is left (load, battery sag).
//
// Finding kS and kF: raise the PWM slowly until the wheel just starts to
// turn — that is kS. Then run at a PWM near full speed, measure the speed in
// ticks/s, and kF = (PWM - kS) / speed.
//
// Wiring: encoder channel A on an interrupt pin; motor driver PWM + DIR.

#include <PIDEasy.h>

#if defined(ARDUINO_ARCH_ESP32)
const uint8_t ENCODER_PIN = 34;
const uint8_t MOTOR_PWM = 25, MOTOR_DIR = 26;
#define ENCODER_ISR_ATTR IRAM_ATTR
#else
const uint8_t ENCODER_PIN = 2;
const uint8_t MOTOR_PWM = 5, MOTOR_DIR = 4;
#define ENCODER_ISR_ATTR
#endif

const unsigned long PERIOD_MS = 20;  // speed is measured over this window
const float TARGET_SPEED = 600.0;    // encoder ticks per second

volatile unsigned long encoderTicks = 0;

void ENCODER_ISR_ATTR onEncoderTick() {
  encoderTicks++;
}

// Only kP and kI: speed readings from an encoder are too coarse for kD.
PIDEasy speedPID(0.15, 1.5, 0.0);

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), onEncoderTick, RISING);

  speedPID.setConstrain(0, 255);          // one direction in this example
  speedPID.setFeedforward(0.35, 25.0);    // kF, kS: measure yours
  speedPID.setIntegralLimit(-60, 60);     // the PI loop only trims
  // Ramp to full power over ~0.5 s: no wheel spin, no brown-out resets.
  speedPID.setOutputRampRate(500);
  digitalWrite(MOTOR_DIR, HIGH);
}

unsigned long lastTicks = 0;
unsigned long lastSample = 0;

void loop() {
  const unsigned long now = millis();
  if (now - lastSample < PERIOD_MS) return;
  const unsigned long dtMs = now - lastSample;
  lastSample = now;

  noInterrupts();
  const unsigned long ticks = encoderTicks;
  interrupts();
  const float speed = (ticks - lastTicks) * 1000.0 / dtMs;
  lastTicks = ticks;

  // We measured dt ourselves, so use the Ms variant.
  const float pwm = speedPID.updateMs(TARGET_SPEED, speed, dtMs);
  analogWrite(MOTOR_PWM, (int)pwm);

  Serial.print(speed);                Serial.print('\t');
  Serial.print(speedPID.getF());      Serial.print('\t');
  Serial.print(speedPID.getI());      Serial.print('\t');
  Serial.println(pwm);
}
