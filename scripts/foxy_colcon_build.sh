#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/foxy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

colcon --log-base log_foxy build \
  --packages-select forklift_msgs forklift_vehicle_interface forklift_nav2_plugins forklift_nav2_demo \
  --symlink-install \
  --build-base build_foxy \
  --install-base install_foxy \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
