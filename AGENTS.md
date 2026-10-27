# Repository Guidelines

## Project Structure & Module Organization

- `src/simulation/robot_mujoco_sim/`: Python ROS 2 node, model loading, CLI, and native/Mjviser viewers. Configuration lives in `src/simulation/config/sim.yaml`; MJCF scenes and meshes live in `src/simulation/robots_model/`.
- `src/z_msgs/msg/`: custom ROS message definitions.
- `src/z_deploy/`: C++ deployment runtime, with headers under `include/z_deploy/`, entry point in `src/main.cpp`, and the Unitree SDK2 submodule under `library/unitree_sdk2/`.
- Tests live in each package's `test/` directory. `scripts/` contains dependency setup helpers. Treat `build/`, `install/`, and `log/` as generated output.

## Build, Test, and Development Commands

Use Ubuntu 22.04, ROS 2 Humble, and system Python 3.10; avoid Conda and `sudo pip`.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pip install --user -r requirements.txt
# Build the simulator and its message dependency:
/usr/bin/python3 /usr/bin/colcon build --symlink-install \
  --packages-up-to robot_mujoco_sim \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 run robot_mujoco_sim robot_mujoco_sim --viewer mjviser
```

Omit `--packages-up-to` to build everything once deployment dependencies are available. `z_deploy` currently builds the simulation backend and uses `z_msgs`; native viewer operation requires a graphical session. Initialize Git submodules before using the Unitree SDK2 headers or libraries.

## Coding Style & Naming Conventions

Use four-space indentation. Python modules, functions, and variables use `snake_case`; classes use `PascalCase`, and constants use `UPPER_SNAKE_CASE`. Follow nearby imports and type annotations.

C++ uses C++17. Follow existing brace placement and local naming; deployment headers use `#pragma once` and `<z_deploy/...>` includes. Prefer lowercase `snake_case` filenames and `Z_` prefixes for project macros. No repository-wide formatter or linter configuration is present.

## Testing Guidelines

Run Python regression tests with pytest installed:

```bash
PYTHONPATH=src/simulation /usr/bin/python3 -m pytest src/simulation/test
colcon test --packages-select z_deploy
colcon test-result --verbose
```

Build `z_deploy` with `-DBUILD_TESTING=ON` before running its CTest-backed tests. Name Python tests `test_*.py`. Add regression coverage for changed behavior, especially viewer locking, callbacks, and shutdown. No numerical coverage threshold is configured.

## Commit & Pull Request Guidelines

Recent commits use short Chinese summaries, such as `修复数据竞争`; no enforced prefix convention is evident. Keep commits focused and summaries specific. PRs should explain behavior changes, list validation commands and results, link relevant issues, and include screenshots for viewer changes. Identify dependency or configuration changes explicitly.
