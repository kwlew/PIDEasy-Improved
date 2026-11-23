// Desktop microbenchmark. Build with the same compiler and flags for each
// revision. The fake clock makes both versions see the same sample times.
#include <chrono>
#include <cstdio>
#include "PIDEasy.h"

uint32_t fake_clock_us = 0;

template <typename Sample>
static void measure(const char* name, PIDEasy& pid, Sample sample) {
  float inputs[256];
  for (int i = 0; i < 256; ++i) {
    inputs[i] = static_cast<float>(i < 128 ? i : 255 - i) - 64.0f;
  }

  const unsigned count = 3000000;
  const auto start = std::chrono::steady_clock::now();
  for (unsigned i = 0; i < count; ++i) sample(pid, inputs[i % 256]);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  const double ns = std::chrono::duration<double, std::nano>(elapsed).count();
  std::printf("%-20s %6.2f ns/sample  output %9.4f\n", name, ns / count, pid.getOutput());
}

int main() {
  const auto compute = [](PIDEasy& pid, float x) { pid.computeMs(x, 10); };
  PIDEasy p(0.75f, 0, 0);
  measure("P", p, compute);
  PIDEasy pi(0.75f, 0.125f, 0);
  measure("PI", pi, compute);
  PIDEasy pid(0.75f, 0.125f, 0.01f);
  measure("PID", pid, compute);

  pid.reset();
  pid.setWindUP(-100, 100);
  pid.setIntegralLimit(-40, 40);
  pid.setDerivativeTimeConstant(0.02f);
  measure("Filtered, limited PID", pid, compute);

  pid.reset();
  pid.setContinuousInput(-180, 180);
  pid.setTolerance(2, 10, 100);
  pid.setOutputRampRate(1000);
  measure("Heading update", pid, [](PIDEasy& p, float x) { p.updateMs(5, x, 10); });

  pi.reset();
  measure("Internally timed PI", pi, [](PIDEasy& p, float x) {
    advanceMicros(10000);
    p.update(5, x);
  });
}
