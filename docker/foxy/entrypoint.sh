#!/usr/bin/env bash
set -e

source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ -f /workspace/install_foxy/setup.bash ] && \
  [ -f /workspace/install_foxy/forklift_msgs/share/forklift_msgs/local_setup.bash ] && \
  [ -f /workspace/install_foxy/forklift_vehicle_interface/share/forklift_vehicle_interface/local_setup.bash ] && \
  [ -f /workspace/install_foxy/forklift_nav2_plugins/share/forklift_nav2_plugins/local_setup.bash ] && \
  [ -f /workspace/install_foxy/forklift_nav2_demo/share/forklift_nav2_demo/local_setup.bash ]; then
  source /workspace/install_foxy/setup.bash
fi

exec "$@"
