#!/usr/bin/env bash
set -euo pipefail
colcon build --symlink-install
source install/setup.bash
ros2 launch tram_reserve_odometry run.py
