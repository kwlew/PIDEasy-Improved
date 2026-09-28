// Host-side test suite for PIDEasy-Improved.
//
// Builds and runs on a desktop compiler against a fake millis() clock, so the
// controller can be regression-tested without flashing a board. See README.md
// in this directory for how to run it.

#include <cstdio>
#include <cmath>
#include "PIDEasy.h"

// Backing store for the fake clock declared in the Arduino.h stub.
uint32_t fake_clock_us = 0;

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

  // The I-term must survive a gain change without the output jumping, so
  // mode switches stay bumpless.
  PID q(0.0f, 1.0f, 0.0f);
  q.computeMs(10.0f, 1000);                     // I-term = 10
  const float before = q.getI();
  q.setTunings(0.0f, 2.0f, 0.0f);               // ki doubles
  const float after = q.computeMs(0.0f, 1000);  // zero error: output unchanged
  check("setTunings is bumpless (ki change keeps the output)",
        near(before, 10.0f) && near(after, 10.0f));
  q.computeMs(1.0f, 1000);                      // new ki applies to new error only
  check("new ki applies to future error only", near(q.getI(), 12.0f));

  q.setTunings(0.0f, 0.0f, 0.0f);
  const float off = q.computeMs(5.0f, 1000);
  check("setTunings with ki = 0 clears the I-term", near(q.getI(), 0.0f) && near(off, 0.0f));
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

  // Both limits are enforced; the tighter one wins. A windup of +/-255 raw,
  // with ki = 0.01, caps the I-term at 2.55 regardless of a looser integral
  // limit.
  PID t(0.0f, 0.01f, 0.0f);
  t.setWindUP(-255.0f, 255.0f);
  t.setIntegralLimit(-60.0f, 60.0f);   // 6000 raw, looser than the windup
  t.setConditionalIntegration(false);
  for (int i = 0; i < 500; i++) t.computeMs(100.0f, 100);
  check("windup limit still applies when tighter", near(t.getI(), 2.55f, 1e-3f));

  // Default: no raw windup clamp, so a small ki is not silently capped at
  // ki * 255. The I-term alone is kept inside the output range instead.
  PID u(0.0f, 0.05f, 0.0f);
  u.setConditionalIntegration(false);
  for (int i = 0; i < 30000; i++) u.computeMs(20.0f, 10);   // 0.01 per step
  check("small ki is not capped by a default windup", u.getI() > 100.0f);
  check("I-term alone never exceeds the output range", near(u.getI(), 255.0f));
  u.setConstrain(-100.0f, 100.0f);
  u.computeMs(0.0f, 10);                                    // no new integration
  check("narrowing the output range re-clamps the I-term", near(u.getI(), 100.0f));
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

  // micros() rollover (every ~71 minutes) must not produce a huge dt. The
  // stub clock is 32 bits wide, so this wraps even on a 64-bit host.
  PID r(0.0f, 1.0f, 0.0f);
  r.setConstrain(-10000.0f, 10000.0f);
  r.setMaxDeltaTime(0);
  setMicros(0xFFFFFFFFUL - 20000UL);
  r.compute(1.0f);
  advanceMicros(40000);         // wraps past zero
  r.compute(1.0f);
  check("micros() rollover yields a sane dt", near(r.getI(), 0.040f, 1e-4f) &&
        near(r.getDeltaTime(), 0.040f, 1e-6f));

  // Diagnostics: a slow loop (over the cap) shows up in wasResumed().
  PID d(0.0f, 1.0f, 0.0f);
  setMillis(0);  d.compute(1.0f);
  check("first sample reports a resume", d.wasResumed());
  setMillis(20); d.compute(1.0f);
  check("normal sample reports its dt", !d.wasResumed() && near(d.getDeltaTime(), 0.020f, 1e-6f));
  d.setMaxDeltaTime(100);
  setMillis(140); d.compute(1.0f);
  check("over-cap gap reports a resume", d.wasResumed() && near(d.getDeltaTime(), 0.100f, 1e-6f));

  // Default cap is 500 ms: a slow 120 ms maze loop must still integrate.
  PID m(0.0f, 1.0f, 0.0f);
  setMillis(0); m.compute(5.0f);
  for (int i = 1; i <= 50; i++) { setMillis(i * 120UL); m.compute(5.0f); }
  check("120 ms loop integrates under the default cap", near(m.getI(), 30.0f, 1e-2f));

  // compute(error) resolves sub-millisecond loop periods with micros().
  PID j(0.0f, 0.0f, 1.0f);
  j.setConstrain(-1e6f, 1e6f);
  setMicros(0); j.compute(0.0f);
  float lo = 1e9f, hi = -1e9f;
  for (int i = 1; i <= 200; i++) {
    setMicros(i * 1500UL);                 // 1.5 ms loop, true rate 100/s
    j.compute(100.0f * i * 0.0015f);
    if (j.getD() < lo) lo = j.getD();
    if (j.getD() > hi) hi = j.getD();
  }
  char buf[64];
  snprintf(buf, sizeof buf, "(D %.2f .. %.2f)", lo, hi);
  check("1.5 ms loop derivative is not quantized", lo > 99.0f && hi < 101.0f, buf);

  setMillis(0);                 // leave the clock tidy for later tests
}

// ---------------------------------------------------------------------------
// Setpoint / measurement API
// ---------------------------------------------------------------------------
static void test_update() {
  section("update(setpoint, measurement)");

  // A heading target step must not kick the D-term.
  PID p(2.0f, 0.0f, 0.5f);
  p.setConstrain(-1e6f, 1e6f);
  p.updateMs(0.0f, 0.0f, 10);
  p.updateMs(90.0f, 0.0f, 10);
  check("setpoint step gives no derivative kick", near(p.getD(), 0.0f));
  check("error is setpoint - measurement", near(p.getP(), 180.0f));

  PID e(2.0f, 0.0f, 0.5f);
  e.setConstrain(-1e6f, 1e6f);
  e.computeMs(0.0f, 10);
  e.computeMs(90.0f, 10);
  check("compute(error) still kicks (for comparison)", near(e.getD(), 4500.0f, 1e-1f));

  // With a fixed setpoint, the measurement derivative matches the error one.
  PID a(0.0f, 0.0f, 1.0f), b(0.0f, 0.0f, 1.0f);
  a.setConstrain(-1e6f, 1e6f);
  b.setConstrain(-1e6f, 1e6f);
  a.updateMs(10.0f, 0.0f, 10);  b.computeMs(10.0f, 10);
  a.updateMs(10.0f, 2.0f, 10);  b.computeMs(8.0f, 10);
  check("fixed setpoint: same D as compute(error)", near(a.getD(), b.getD()) && near(a.getD(), -200.0f));

  // Internal timer is shared with compute(error).
  PID t(0.0f, 1.0f, 0.0f);
  setMillis(0);  t.update(1.0f, 0.0f);
  setMillis(50); t.update(1.0f, 0.0f);
  check("update() uses the internal timer", near(t.getI(), 0.050f, 1e-4f));

  PID z(1.0f, 0.0f, 0.0f);
  z.updateMs(5.0f, 0.0f, 10);
  check("updateMs with dt = 0 holds", near(z.updateMs(50.0f, 0.0f, 0), 5.0f));
  check("NaN measurement holds", near(z.updateMs(5.0f, nanf(""), 10), 5.0f));

  PID u(0.0f, 1.0f, 0.0f);
  u.updateUs(1.0f, 0.0f, 250);
  check("updateUs integrates microseconds", near(u.getI(), 0.00025f, 1e-7f));
  PID c(0.0f, 1.0f, 0.0f);
  c.computeUs(1.0f, 250);
  check("computeUs integrates microseconds", near(c.getI(), 0.00025f, 1e-7f) &&
        near(c.computeUs(9.0f, 0), c.getOutput()));

  setMillis(0);
}

// ---------------------------------------------------------------------------
// Robustness: non-finite input, sub-millisecond loops
// ---------------------------------------------------------------------------
static void test_robustness() {
  section("non-finite error / sub-millisecond loop");

  // A line position computed as 0/0 (all sensors white) must not latch NaN
  // into the integral and kill the controller for the rest of the run.
  const float nan_error = nanf("");
  const float inf_error = INFINITY;
  PID p(1.0f, 1.0f, 1.0f);
  p.setConstrain(-1000.0f, 1000.0f);
  p.computeMs(5.0f, 10);
  const float held = p.computeMs(5.0f, 10);
  check("NaN error holds the last output", near(p.computeMs(nan_error, 10), held));
  check("infinite error holds the last output", near(p.computeMs(inf_error, 10), held));
  const float after = p.computeMs(5.0f, 10);
  check("controller keeps working after NaN / inf", after == after && near(after, held + 0.05f, 1e-3f));

  // NaN through the millis() path must not consume elapsed time.
  PID q(0.0f, 1.0f, 0.0f);
  q.setConstrain(-1000.0f, 1000.0f);
  setMillis(0);  q.compute(1.0f);
  setMillis(20); q.compute(nan_error);
  setMillis(40); q.compute(1.0f);
  check("NaN sample does not swallow elapsed time", near(q.getI(), 0.040f, 1e-4f));

  // Four calls per millisecond: the I-term must integrate real time, not
  // one forced millisecond per call.
  PID r(0.0f, 1.0f, 0.0f);
  r.setConstrain(-1000.0f, 1000.0f);
  setMillis(0);
  r.compute(1.0f);
  for (unsigned long ms = 1; ms <= 1000; ms++) {
    setMillis(ms);
    for (int k = 0; k < 4; k++) r.compute(1.0f);
  }
  char buf[64];
  snprintf(buf, sizeof buf, "(I = %.3f)", r.getI());
  check("4 kHz loop integrates real time, not per call", near(r.getI(), 1.0f, 1e-3f), buf);

  // The first compute(error) call has no elapsed time to integrate.
  PID s(0.0f, 1.0f, 0.0f);
  setMillis(0);
  s.compute(10.0f);
  check("first compute(error) does not integrate", near(s.getI(), 0.0f));

  setMillis(0);
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
  test_robustness();
  test_update();
  test_derivative_filter();
  test_backwards_compatibility();

  printf("\n%s — %d checks, %d failure%s\n",
         failures ? "FAILURES" : "ALL PASS", checks, failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
