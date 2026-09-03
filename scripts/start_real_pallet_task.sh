#!/usr/bin/env bash
set -Eeuo pipefail

workspace_dir="${WORKSPACE_DIR:-/workspace}"
map_file="${MAP_FILE:-map3.yaml}"
nav2_params_file="${NAV2_PARAMS_FILE:-src/forklift_nav2_demo/config/forklift_nav2_real_external_localization_foxy.yaml}"
vehicle_dry_run="${VEHICLE_DRY_RUN:-false}"
can_interface="${CAN_INTERFACE:-can0}"
task_start_delay_sec="${TASK_START_DELAY_SEC:-4}"

cd "$workspace_dir"

if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
  echo "ROS 2 Foxy setup was not found." >&2
  exit 2
fi
if [[ ! -f install/setup.bash ]]; then
  echo "Workspace is not built: $workspace_dir/install/setup.bash is missing." >&2
  exit 2
fi
if [[ ! -f "$map_file" ]]; then
  echo "Map YAML was not found: $workspace_dir/$map_file" >&2
  exit 2
fi
if [[ ! -f "$nav2_params_file" ]]; then
  echo "Nav2 parameter file was not found: $workspace_dir/$nav2_params_file" >&2
  exit 2
fi
if [[ "$vehicle_dry_run" != true && "$vehicle_dry_run" != false ]]; then
  echo "VEHICLE_DRY_RUN must be true or false." >&2
  exit 2
fi

set +u
source /opt/ros/foxy/setup.bash
source install/setup.bash
set -u

nav_pid=""
task_pid=""

shutdown() {
  trap - EXIT INT TERM
  if [[ -n "$task_pid" ]] && kill -0 "$task_pid" 2>/dev/null; then
    kill -INT "$task_pid" 2>/dev/null || true
  fi
  if [[ -n "$nav_pid" ]] && kill -0 "$nav_pid" 2>/dev/null; then
    kill -INT "$nav_pid" 2>/dev/null || true
  fi
  [[ -z "$task_pid" ]] || wait "$task_pid" 2>/dev/null || true
  [[ -z "$nav_pid" ]] || wait "$nav_pid" 2>/dev/null || true
}

trap 'rc=$?; shutdown; exit "$rc"' EXIT
trap 'shutdown; exit 130' INT TERM

echo "Starting real navigation: map=$map_file dry_run=$vehicle_dry_run"
ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py \
  map:="$map_file" \
  nav2_params_file:="$nav2_params_file" \
  vehicle_dry_run:="$vehicle_dry_run" \
  can_interface:="$can_interface" \
  use_rviz:=false \
  use_localization_adapter:=true \
  localization_topic:=/fusion/localization \
  odom_topic:=/odom \
  localization_base_frame_id:=base_link \
  odom_base_frame_id:=base_footprint \
  base_frame_id:=base_link \
  localization_adapter_stamp_with_current_time:=true \
  localization_adapter_publish_rate_hz:=20.0 \
  localization_offset_x_m:=0.0 \
  localization_offset_y_m:=0.0 \
  localization_offset_yaw_rad:=0.0 \
  invert_drive_direction:=true \
  use_fine_motion_adapter:=true \
  invert_steering_angle:=false &
nav_pid=$!

sleep "$task_start_delay_sec"
if ! kill -0 "$nav_pid" 2>/dev/null; then
  echo "Navigation exited before task manager startup." >&2
  wait "$nav_pid"
fi

echo "Starting pallet task manager. RViz /goal_pose is now a pallet target."
ros2 launch forklift_task_manager pallet_approach.launch.py \
  use_sim_time:=false &
task_pid=$!

echo "Navigation PID: $nav_pid; task manager PID: $task_pid"
echo "Press Ctrl-C to stop both processes."

set +e
wait -n "$nav_pid" "$task_pid"
status=$?
set -e
echo "A launch process exited with status $status; stopping the other process."
exit "$status"
