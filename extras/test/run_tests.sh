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

"$DIR/test_pideasy"
