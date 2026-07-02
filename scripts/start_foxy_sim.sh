#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

map_file="${MAP_FILE:-/workspace/forklift_factory_big_map_clean.yaml}"
params_file="${NAV2_PARAMS_FILE:-/workspace/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml}"
run_ab_test="${RUN_AB_TEST:-0}"

case "$run_ab_test" in
  0|1) ;;
  *)
    echo "RUN_AB_TEST must be 0 or 1." >&2
    exit 2
    ;;
esac

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Run this script from the graphical desktop terminal." >&2
  exit 2
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not in PATH." >&2
  exit 2
fi

if ! docker image inspect forklift-nav2:foxy >/dev/null 2>&1; then
  echo "Foxy image forklift-nav2:foxy was not found." >&2
  echo "Build it first with: ./scripts/foxy_docker_build.sh" >&2
  exit 2
fi

if [[ ! -f install_foxy/setup.bash ]]; then
  echo "Foxy workspace has not been built." >&2
  echo "Build it first with:" >&2
  echo "  ./scripts/foxy_docker_run.sh bash -lc 'export CCACHE_DIR=/tmp/ccache CCACHE_TEMPDIR=/tmp/ccache-tmp; ./scripts/foxy_colcon_build.sh'" >&2
  exit 2
fi

# The container uses the host UID. Grant only that local user access to X11,
# then remove the rule when the simulation exits.
xhost_rule_added=0
if command -v xhost >/dev/null 2>&1; then
  if xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1; then
    xhost_rule_added=1
  fi
fi

cleanup_xhost() {
  if [[ "$xhost_rule_added" == 1 ]]; then
    xhost "-SI:localuser:$(id -un)" >/dev/null 2>&1 || true
  fi
}
trap cleanup_xhost EXIT

echo "Starting ROS 2 Foxy simulation"
echo "  map: $map_file"
echo "  Gazebo GUI: enabled"
echo "  RViz: enabled"
echo "  initial pose A: (-2.0, -0.5, 0.0)"
if [[ "$run_ab_test" == 1 ]]; then
  echo "  automatic A->B test: enabled, B=(1.2, -0.5, 0.0)"
else
  echo "  automatic A->B test: disabled"
  echo "  Set a goal with RViz '2D Goal Pose', or use RUN_AB_TEST=1."
fi
echo "Press Ctrl-C to stop the complete simulation."

FOXY_CONTAINER_NAME=forklift-foxy-sim \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ./scripts/foxy_docker_run.sh bash -lc '
set -euo pipefail

map_file="$1"
params_file="$2"
run_ab_test="$3"

set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch forklift_nav2_demo forklift_navigation.launch.py \
  map:="$map_file" \
  nav2_params_file:="$params_file" \
  use_sim_time:=true \
  use_rviz:=true \
  gazebo_gui:=true \
  use_sim_command_bridge:=true \
  use_safety_command_gate:=true \
  safety_costmap_timeout_sec:=1.5 \
  nav2_start_delay:=5.0 \
  sim_ready_timeout:=30.0 \
  rmw_implementation:=rmw_cyclonedds_cpp \
  use_composition:=False &
launch_pid=$!

stop_launch() {
  kill -INT "$launch_pid" >/dev/null 2>&1 || true
  wait "$launch_pid" >/dev/null 2>&1 || true
}
trap stop_launch INT TERM EXIT

# Wait until Nav2 exposes its top-level action before initializing AMCL.
for _ in $(seq 1 90); do
  if ros2 action list 2>/dev/null | grep -qx /navigate_to_pose; then
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid"
    exit $?
  fi
  sleep 1
done

if [[ "$run_ab_test" == 1 ]]; then
  ros2 run forklift_nav2_demo forklift_ab_acceptance \
    --ros-args -p use_sim_time:=true -p scenario:=forward_ab || true
else
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
fi

wait "$launch_pid"
' _ "$map_file" "$params_file" "$run_ab_test"
