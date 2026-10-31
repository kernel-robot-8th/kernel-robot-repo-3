#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${repo_dir}/ros2_ws"

colcon build --symlink-install --packages-select toktak_warehouse
set +u
source install/setup.bash
set -u
ros2 launch toktak_warehouse toktak_warehouse.launch.py "$@"
