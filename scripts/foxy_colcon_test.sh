#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

colcon --log-base log_foxy test \
  --packages-select forklift_nav2_plugins \
  --build-base build_foxy \
  --install-base install_foxy \
  --event-handlers console_cohesion+

colcon --log-base log_foxy test-result \
  --test-result-base build_foxy \
  --verbose
