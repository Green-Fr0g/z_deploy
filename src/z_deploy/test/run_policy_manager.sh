#!/usr/bin/env bash
# Run the policy/FSM interface regression test without ROS or inference runtimes.
set -euo pipefail

policy_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
policy_test_dir="$(mktemp -d /tmp/frog-policy-check-XXXXXX)"
trap 'rm -rf -- "$policy_test_dir"' EXIT

# core/ and include/ are used directly as header search roots.
for test_name in test_policy_manager test_fsm; do
    "${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -pthread \
        -DZ_DEPLOY_HAS_TORCH=0 \
        -I"$policy_repo_dir/core" -I"$policy_repo_dir/include" \
        "$policy_repo_dir/test/$test_name.cpp" -lyaml-cpp \
        -o "$policy_test_dir/$test_name"
    "$policy_test_dir/$test_name"
done
