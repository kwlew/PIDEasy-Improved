// Host-side test suite for PIDEasy-Improved.
//
// Builds and runs on a desktop compiler against a fake millis() clock, so the
// controller can be regression-tested without flashing a board. See README.md
// in this directory for how to run it.

#include <cstdio>
#include <cmath>
#include "PIDEasy.h"

// Backing store for the fake clock declared in the Arduino.h stub.
unsigned long fake_clock_ms = 0;

static int failures = 0;
static int checks = 0;

static void check(const char* name, bool ok, const char* detail = "") {
  checks++;
  printf("%-52s %s %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

static bool near(float a, float b, float tol = 1e-4f) { return fabsf(a - b) <= tol; }

static void section(const char* title) { printf("\n-- %s\n", title); }

// ---------------------------------------------------------------------------
// Runtime gain changes
// ---------------------------------------------------------------------------
static void test_tunings() {
  section("setTunings / gain getters");

  PID p(1.0f, 2.0f, 3.0f);
  check("getKp/getKi/getKd return constructor gains",
        near(p.getKp(), 1.0f) && near(p.getKi(), 2.0f) && near(p.getKd(), 3.0f));

  p.setTunings(4.0f, 5.0f, 6.0f);
  check("setTunings updates all three gains",
        near(p.getKp(), 4.0f) && near(p.getKi(), 5.0f) && near(p.getKd(), 6.0f));

  // The integral must survive a gain change so mode switches stay bumpless.
  PID q(0.0f, 1.0f, 0.0f);
  q.computeMs(10.0f, 1000);                     // integral = 10
  const float before = q.getI();                // ki*I = 10
  q.setTunings(0.0f, 2.0f, 0.0f);               // ki doubles, integral kept
  const float after = q.computeMs(0.0f, 1000);  // integral still 10 -> out 20
  check("setTunings preserves the integral", near(before, 10.0f) && near(after, 20.0f));
}

// ---------------------------------------------------------------------------
// Per-term telemetry
// ---------------------------------------------------------------------------
static void test_telemetry() {
  section("getP / getI / getD / getOutput");

  PID p(2.0f, 1.0f, 0.5f);
  p.setConstrain(-1000.0f, 1000.0f);
  const float out = p.computeMs(4.0f, 1000);    // dt = 1 s, integral = 4, d = 0
  check("getP == kp*error", near(p.getP(), 8.0f));
  check("getI == ki*integral", near(p.getI(), 4.0f));
  check("getD == 0 on first sample", near(p.getD(), 0.0f));
  check("P+I+D == returned output", near(p.getP() + p.getI() + p.getD(), out));
  check("getOutput matches last return", near(p.getOutput(), out));

  const float out2 = p.computeMs(6.0f, 1000);   // d = (6-4)/1 = 2, kd*d = 1.0
  check("getD tracks derivative on later samples", near(p.getD(), 1.0f));
  check("getOutput updates each call", near(p.getOutput(), out2));

  p.reset();
  check("reset clears telemetry",
        near(p.getP(), 0.0f) && near(p.getI(), 0.0f) &&
        near(p.getD(), 0.0f) && near(p.getOutput(), 0.0f));
}

// ---------------------------------------------------------------------------
// Anti-windup: output-unit integral limit
// ---------------------------------------------------------------------------
static void test_integral_limit() {
  section("setIntegralLimit");

  PID p(0.0f, 0.01f, 0.0f);
  p.setWindUP(-10000.0f, 10000.0f);    // deliberately loose
  p.setIntegralLimit(-50.0f, 50.0f);   // I may contribute at most +/-50
  p.setConstrain(-255.0f, 255.0f);
  p.setConditionalIntegration(false);  // isolate the clamp
  for (int i = 0; i < 500; i++) p.computeMs(100.0f, 100);
  check("integral limit caps I-term in output units", near(p.getI(), 50.0f, 1e-3f));

  // The whole point: the limit must not shift when ki is retuned.
  p.setTunings(0.0f, 0.05f, 0.0f);
  for (int i = 0; i < 500; i++) p.computeMs(100.0f, 100);
  check("integral limit survives a ki change", near(p.getI(), 50.0f, 1e-3f));

  p.setIntegralLimit(0.0f, 0.0f);      // disable -> falls back to windup
  for (int i = 0; i < 500; i++) p.computeMs(100.0f, 100);
  check("setIntegralLimit(0,0) disables the limit", p.getI() > 100.0f);

  PID s(0.0f, 0.01f, 0.0f);
  s.setWindUP(-10000.0f, 10000.0f);    // must not be the binding limit here
  s.setIntegralLimit(60.0f, -60.0f);   // swapped on purpose
  s.setConditionalIntegration(false);
  // 60 / ki = 6000 raw, and each step adds 10, so this needs > 600 steps.
  for (int i = 0; i < 1000; i++) s.computeMs(100.0f, 100);
  check("setIntegralLimit swaps reversed arguments", near(s.getI(), 60.0f, 1e-3f));

  // Both limits are enforced; the tighter one wins. Default windup is +/-255
  // raw, which with ki = 0.01 caps the I-term at 2.55 regardless of a looser
  // integral limit.
  PID t(0.0f, 0.01f, 0.0f);
  t.setIntegralLimit(-60.0f, 60.0f);   // 6000 raw, looser than default windup
  t.setConditionalIntegration(false);
  for (int i = 0; i < 500; i++) t.computeMs(100.0f, 100);
  check("windup limit still applies when tighter", near(t.getI(), 2.55f, 1e-3f));
}

// ---------------------------------------------------------------------------
// Anti-windup: conditional integration
// ---------------------------------------------------------------------------
static void test_conditional_integration() {
  section("setConditionalIntegration");

  const float e_hi = 100.0f, e_lo = -5.0f;

  PID with(2.0f, 1.0f, 0.0f);
  with.setConstrain(-255.0f, 255.0f);
  PID without(2.0f, 1.0f, 0.0f);
  without.setConstrain(-255.0f, 255.0f);
  without.setConditionalIntegration(false);

  for (int i = 0; i < 100; i++) { with.computeMs(e_hi, 50); without.computeMs(e_hi, 50); }
  check("conditional integration holds the integral down", with.getI() < without.getI());

  // Flip the error and count cycles until the output actually reverses.
  int recover_with = -1, recover_without = -1;
  for (int i = 0; i < 500; i++) {
    const float a = with.computeMs(e_lo, 50);
    const float b = without.computeMs(e_lo, 50);
    if (recover_with < 0 && a < 0.0f) recover_with = i;
    if (recover_without < 0 && b < 0.0f) recover_without = i;
  }
  // -1 means it never recovered inside the window, i.e. the worst case.
  char buf[96];
  snprintf(buf, sizeof buf, "(%d vs %s cycles)", recover_with,
           recover_without < 0 ? "never" : "sooner-check");
  check("saturated output recovers sooner with it on",
        recover_with >= 0 && (recover_without < 0 || recover_with < recover_without), buf);

  // Integration that unwinds saturation must never be blocked.
  PID q(0.0f, 1.0f, 0.0f);
  q.setConstrain(-10.0f, 10.0f);
  for (int i = 0; i < 50; i++) q.computeMs(5.0f, 100);   // saturate high
  const float pinned = q.getI();
  q.computeMs(-5.0f, 100);                               // error flips: must integrate
  check("integration still allowed when it unwinds saturation", q.getI() < pinned);
}

// ---------------------------------------------------------------------------
// dt measurement, cap and resume
// ---------------------------------------------------------------------------
static void test_dt_and_resume() {
  section("compute(error) dt measurement / resume");

  PID p(0.0f, 1.0f, 1.0f);
  p.setConstrain(-10000.0f, 10000.0f);

  setMillis(1000);
  p.compute(5.0f);              // first call initializes the timer
  advanceMillis(50);            // 50 ms, inside the 100 ms default cap
  p.compute(5.0f);
  const float i_before = p.getI();

  advanceMillis(5000);          // 5 s gap -> resume sample
  p.compute(5.0f);
  check("resume sample skips the integral step", near(p.getI(), i_before));
  check("resume sample zeroes the derivative", near(p.getD(), 0.0f));

  advanceMillis(50);            // back to a normal cycle
  p.compute(5.0f);
  check("integration resumes on the next normal sample", p.getI() > i_before);

  // Cap disabled -> a long gap integrates the whole interval again.
  PID q(0.0f, 1.0f, 0.0f);
  q.setConstrain(-10000.0f, 10000.0f);
  q.setMaxDeltaTime(0);
  setMillis(0);    q.compute(1.0f);
  setMillis(5000); q.compute(1.0f);
  check("setMaxDeltaTime(0) disables the cap", q.getI() > 4.0f);

  // millis() rollover must not produce a huge or negative dt.
  PID r(0.0f, 1.0f, 0.0f);
  r.setConstrain(-10000.0f, 10000.0f);
  r.setMaxDeltaTime(0);
  setMillis(0xFFFFFFFFUL - 20UL);
  r.compute(1.0f);
  advanceMillis(40);            // wraps past zero
  r.compute(1.0f);
  check("millis() rollover yields a sane dt", near(r.getI(), 0.040f, 1e-3f));

  setMillis(0);                 // leave the clock tidy for later tests
}

// ---------------------------------------------------------------------------
// Derivative filtering: fixed coefficient vs time constant
// ---------------------------------------------------------------------------

// Drive a constant error ramp (true derivative == rate) for a fixed amount of
// WALL-CLOCK time, split into `steps` samples of dt_ms each, and return the
// filtered derivative that came out. kd = 1, so getD() is that derivative.
static float filtered_after(unsigned long dt_ms, int steps, bool tau_mode, float param) {
  PID p(0.0f, 0.0f, 1.0f);
  p.setConstrain(-1e6f, 1e6f);
  if (tau_mode) p.setDerivativeTimeConstant(param);
  else          p.setSmoothingDerivative(param);

  const float dt = dt_ms / 1000.0f;
  const float rate = 100.0f;              // true derivative of the ramp
  float error = 0.0f;
  p.computeMs(error, dt_ms);              // prime: first sample forces d = 0
  for (int i = 0; i < steps; i++) {
    error += rate * dt;
    p.computeMs(error, dt_ms);
  }
  return p.getD();
}

static void test_derivative_filter() {
  section("setDerivativeTimeConstant");

  // Same 200 ms of wall clock, two different loop rates.
  const float fixed_fast = filtered_after(10, 20, false, 0.9f);
  const float fixed_slow = filtered_after(50, 4,  false, 0.9f);
  const float tau_fast   = filtered_after(10, 20, true,  0.1f);
  const float tau_slow   = filtered_after(50, 4,  true,  0.1f);

  char buf[128];
  snprintf(buf, sizeof buf, "(%.1f vs %.1f)", fixed_fast, fixed_slow);
  check("fixed coefficient IS loop-rate dependent (the bug)",
        fabsf(fixed_fast - fixed_slow) > 0.3f * fixed_fast, buf);

  snprintf(buf, sizeof buf, "(%.1f vs %.1f)", tau_fast, tau_slow);
  check("time constant is loop-rate independent",
        fabsf(tau_fast - tau_slow) < 0.10f * tau_fast, buf);

  // Both should approach the analytic 1 - exp(-t/tau) = 86.5% of the ramp rate.
  const float expected = 100.0f * (1.0f - expf(-0.200f / 0.100f));
  snprintf(buf, sizeof buf, "(expected ~%.1f)", expected);
  check("tau filter tracks the analytic step response",
        fabsf(tau_fast - expected) < 0.10f * expected, buf);

  // tau = 0 disables filtering: output is the raw derivative.
  PID p(0.0f, 0.0f, 1.0f);
  p.setConstrain(-1e6f, 1e6f);
  p.setDerivativeTimeConstant(0.0f);
  p.computeMs(0.0f, 100);
  p.computeMs(10.0f, 100);                // raw derivative = 10 / 0.1 = 100
  check("setDerivativeTimeConstant(0) disables filtering", near(p.getD(), 100.0f, 1e-2f));

  // The two modes are mutually exclusive, last call wins, in both directions.
  PID q(0.0f, 0.0f, 1.0f);
  q.setConstrain(-1e6f, 1e6f);
  q.setDerivativeTimeConstant(0.5f);
  q.setSmoothingDerivative(0.0f);         // back to fixed, no smoothing
  q.computeMs(0.0f, 100);
  q.computeMs(10.0f, 100);
  check("setSmoothingDerivative overrides tau mode", near(q.getD(), 100.0f, 1e-2f));

  PID r(0.0f, 0.0f, 1.0f);
  r.setConstrain(-1e6f, 1e6f);
  r.setSmoothingDerivative(0.9f);
  r.setDerivativeTimeConstant(0.0f);      // tau 0 clears the fixed value too
  r.computeMs(0.0f, 100);
  r.computeMs(10.0f, 100);
  check("setDerivativeTimeConstant(0) overrides fixed smoothing",
        near(r.getD(), 100.0f, 1e-2f));
}

// ---------------------------------------------------------------------------
// Backwards compatibility with 1.0.x behavior
// ---------------------------------------------------------------------------
static void test_backwards_compatibility() {
  section("backwards compatibility");

  PID p(1.0f, 0.0f, 0.0f);
  check("compute(error, dt) seconds overload unchanged", near(p.compute(10.0f, 2UL), 10.0f));

  PID q(0.0f, 1.0f, 0.0f);
  check("computeMs still integrates in ms", near(q.computeMs(10.0f, 500), 5.0f));

  PID r(1.0f, 0.0f, 0.0f);
  r.setConstrain(-5.0f, 5.0f);
  check("output constrain still applied", near(r.computeMs(100.0f, 100), 5.0f));

  PID s(0.0f, 1.0f, 0.0f);
  s.setDampingFactor(0.5f);
  s.computeMs(10.0f, 1000);              // integral = 10
  s.computeMs(-2.0f, 1000);              // sign flip: (10 - 2) * 0.5 = 4
  check("sign-change damping unchanged", near(s.getI(), 4.0f));

  PID t(0.0f, 0.0f, 1.0f);
  t.setConstrain(-1e6f, 1e6f);
  t.setSmoothingDerivate(0.5f);          // misspelled legacy alias
  t.computeMs(0.0f, 100);
  t.computeMs(10.0f, 100);
  check("setSmoothingDerivate alias still works", near(t.getD(), 50.0f, 1e-2f));

  PID u(1.0f, 0.0f, 0.0f);
  u.setConstrain(10.0f, -10.0f);         // swapped on purpose
  check("setConstrain swaps reversed arguments", near(u.computeMs(100.0f, 100), 10.0f));
}

int main() {
  printf("PIDEasy-Improved host test suite\n");

  test_tunings();
  test_telemetry();
  test_integral_limit();
  test_conditional_integration();
  test_dt_and_resume();
  test_derivative_filter();
  test_backwards_compatibility();

  printf("\n%s — %d checks, %d failure%s\n",
         failures ? "FAILURES" : "ALL PASS", checks, failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
