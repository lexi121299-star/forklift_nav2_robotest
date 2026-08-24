#!/usr/bin/env bash
set -euo pipefail

container_name="${FOXY_REAL_CONTAINER_NAME:-forklift-foxy-real}"
max_retries="${TASK_MAX_RETRIES:-1}"
stations_file="${TASK_STATIONS_FILE:-}"
routes_file="${TASK_ROUTES_FILE:-}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not in PATH." >&2
  exit 2
fi

if ! docker container inspect "$container_name" >/dev/null 2>&1; then
  echo "Real-vehicle container '$container_name' does not exist." >&2
  echo "Start it first with: ./scripts/start_foxy_real.sh" >&2
  exit 2
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$container_name")" != true ]]; then
  echo "Real-vehicle container '$container_name' is not running." >&2
  echo "Start it first with: ./scripts/start_foxy_real.sh" >&2
  exit 2
fi

if ! [[ "$max_retries" =~ ^[0-9]+$ ]]; then
  echo "TASK_MAX_RETRIES must be a non-negative integer." >&2
  exit 2
fi

launch_args=(
  "use_sim_time:=false"
  "max_retries:=$max_retries"
)
if [[ -n "$stations_file" ]]; then
  launch_args+=("stations_file:=$stations_file")
fi
if [[ -n "$routes_file" ]]; then
  launch_args+=("routes_file:=$routes_file")
fi

echo "Starting forklift_task_manager in '$container_name'."
echo "In RViz, use 2D Goal Pose: click the target and drag the arrow to set the final fork direction."
echo "Press Ctrl-C here to stop only the task manager; the real-vehicle navigation stack keeps running."

exec docker exec -it "$container_name" bash -lc '
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u
exec ros2 launch forklift_task_manager task_manager.launch.py "$@"
' _ "${launch_args[@]}"
