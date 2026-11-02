#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d)
trap 'rm -rf -- "$test_dir"' EXIT
# Stage the unchanged public header so quoted SDK includes resolve to fakes.
cp include/lift.hpp "$test_dir/lift.hpp"
g++ -std=c++17 -Wall -Wextra -Werror -Itests/lift_fakes -I"$test_dir" -Iinclude \
    tests/lift_worker_test.cpp src/lift.cpp -o "$test_dir/worker"
for gearing in 0 1 2 3; do "$test_dir/worker" "$gearing"; done
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/lift_feedback_test.cpp -o "$test_dir/feedback"
"$test_dir/feedback"
echo 'Lift worker and feedback tests passed.'
