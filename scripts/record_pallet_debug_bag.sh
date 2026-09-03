#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="${WORKSPACE_DIR:-/workspace}"
bag_root="${BAG_OUTPUT_DIR:-$HOME/ros2bag}"

cd "$workspace_dir"

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy setup was not found." >&2
  exit 2
fi
if [[ ! -f install/setup.bash ]]; then
  echo "Workspace is not built: $workspace_dir/install/setup.bash is missing." >&2
  exit 2
fi

set +u
source /opt/ros/foxy/setup.bash
source install/setup.bash
set -u

mkdir -p "$bag_root"
bag_path="$bag_root/pallet_debug_$(date +%Y%m%d_%H%M%S)"
navigate_to_pose_action="${NAVIGATE_TO_POSE_ACTION:-/navigate_to_pose}"

topics=(
  /rosout
  /tf
  /tf_static
  /map
  /map_metadata
  /odom
  /fusion/localization
  /scan
  /goal_pose
  /plan
  /cmd_vel
  /forklift/vehicle_state
  /forklift/fault_state
  /forklift/fork/joint_state
  /forklift/control_cmd_raw
  /forklift/control_cmd
  /forklift/safety_gate/status
  /forklift/task_status
  /global_costmap/costmap_raw
  /local_costmap/costmap_raw
  /global_costmap/published_footprint
  /local_costmap/published_footprint
  /forklift/pallet_approach/staging_runup_pose
  /forklift/pallet_approach/staging_pose
  /forklift/pallet_approach/alignment_pose
  /forklift/pallet_approach/pre_approach_pose
  /forklift/pallet_approach/stop_pose
  /forklift/pallet_approach/exemption_pose
  /forklift/pallet_approach/exemption_active
  "$navigate_to_pose_action/_action/status"
  "$navigate_to_pose_action/_action/feedback"
  /compute_path_to_pose/_action/status
  /compute_path_to_pose/_action/feedback
  /follow_path/_action/status
  /follow_path/_action/feedback
  /forklift/fine_motion/move_relative/_action/status
  /forklift/fine_motion/move_relative/_action/feedback
  /forklift/fine_motion/pivot_relative/_action/status
  /forklift/fine_motion/pivot_relative/_action/feedback
)

echo "Recording pallet debug bag to: $bag_path"
echo "Press Ctrl-C after the test to finish the bag cleanly."
exec ros2 bag record --include-hidden-topics -o "$bag_path" "${topics[@]}"
