#!/usr/bin/env bash
# Run the policy/FSM interface regression test without ROS or inference runtimes.
set -euo pipefail

policy_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
policy_test_dir="$(mktemp -d /tmp/frog-policy-check-XXXXXX)"
trap 'rm -rf -- "$policy_test_dir"' EXIT

# Expose the current core/ and include/ layout under its C++ include namespace.
mkdir -p "$policy_test_dir/include/z_deploy"
for module in "$policy_repo_dir"/core/*; do
    ln -s "$module" "$policy_test_dir/include/z_deploy/"
done
for header in "$policy_repo_dir"/include/*.hpp; do
    ln -s "$header" "$policy_test_dir/include/z_deploy/"
done

for test_name in test_policy_manager test_fsm; do
    "${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -pthread \
        -DZ_DEPLOY_HAS_TORCH=0 -I"$policy_test_dir/include" \
        "$policy_repo_dir/test/$test_name.cpp" -lyaml-cpp \
        -o "$policy_test_dir/$test_name"
    "$policy_test_dir/$test_name"
done
