#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_tb3_ros_env.bash"

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
