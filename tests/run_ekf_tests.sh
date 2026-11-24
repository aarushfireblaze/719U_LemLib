#!/usr/bin/env bash
# Off-robot tests for the EKF localization math. No PROS needed.
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d)
trap 'rm -rf -- "$test_dir"' EXIT
flags=(-std=c++17 -Wall -Wextra -Werror -Iinclude)
g++ "${flags[@]}" tests/ekf_test.cpp src/localization/ekf.cpp -o "$test_dir/ekf"
"$test_dir/ekf"
g++ "${flags[@]}" tests/field_map_test.cpp src/localization/field_map.cpp \
    src/localization/distance_sensor_model.cpp src/localization/ekf.cpp -o "$test_dir/field_map"
"$test_dir/field_map"
echo "EKF tests passed."
