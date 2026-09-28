// Compile-only check: with PIDEASY_NO_PID_ALIAS defined, PIDEasy.h must not
// claim the name PID, so a sketch can also use another library that defines
// a class PID (e.g. PID_v1). run_tests.sh compiles this with -fsyntax-only.

#define PIDEASY_NO_PID_ALIAS
#include "PIDEasy.h"

// Stand-in for another library's class of the same name.
class PID {
  public:
    explicit PID(double* input) : input(input) {}
    double* input;
};

static double reading = 0.0;
static PID other(&reading);
static PIDEasy mine(1.0f, 0.0f, 0.0f);

float use_both() { return mine.computeMs(static_cast<float>(*other.input), 10); }
