#!/bin/sh
set -eu
cd "$(dirname "$0")"
# Fast-math is intentionally disabled to preserve finite-state checks and equations.
g++ -O3 -std=c++17 -fopenmp native_accel.cpp -o native_accel
