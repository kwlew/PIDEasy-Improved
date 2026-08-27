#!/bin/sh
# Build and run the PIDEasy-Improved host test suite.
# Usage: ./run_tests.sh   (from anywhere)
set -e

DIR=$(cd "$(dirname "$0")" && pwd)
SRC="$DIR/../../src"
CXX=${CXX:-g++}

"$CXX" -std=c++11 -Wall -Wextra -I"$DIR" -I"$SRC" \
    "$DIR/test_pideasy.cpp" "$SRC/PIDEasy.cpp" -o "$DIR/test_pideasy"

"$DIR/test_pideasy"
