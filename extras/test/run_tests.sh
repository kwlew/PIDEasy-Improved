#!/bin/sh
# Build and run the PIDEasy-Improved host test suite.
# Usage: ./run_tests.sh   (from anywhere)
# Extra compiler flags can be passed in CXXFLAGS, e.g. sanitizers in CI.
set -e

DIR=$(cd "$(dirname "$0")" && pwd)
SRC="$DIR/../../src"
CXX=${CXX:-g++}

# shellcheck disable=SC2086  # CXXFLAGS is intentionally word-split
"$CXX" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$DIR" -I"$SRC" \
    "$DIR/test_pideasy.cpp" "$SRC/PIDEasy.cpp" -o "$DIR/test_pideasy"

# The PIDEASY_NO_PID_ALIAS opt-out must leave the name PID free.
# shellcheck disable=SC2086
"$CXX" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -fsyntax-only -I"$DIR" -I"$SRC" \
    "$DIR/no_alias_check.cpp"

"$DIR/test_pideasy"
