#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

map_file="${MAP_FILE:-/workspace/forklift_factory_big_map_clean.yaml}"
params_file="${NAV2_PARAMS_FILE:-/workspace/forklift_nav2_demo/config/forklift_nav2_oru_test_foxy.yaml}"
vehicle_dry_run="${VEHICLE_DRY_RUN:-true}"
can_interface="${CAN_INTERFACE:-can0}"
use_rviz="${USE_RVIZ:-true}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not in PATH." >&2
  exit 2
fi
if ! docker image inspect forklift-nav2:foxy >/dev/null 2>&1; then
  echo "Foxy image forklift-nav2:foxy was not found." >&2
  exit 2
fi
if [[ ! -f install_foxy/setup.bash ]]; then
  echo "Foxy workspace has not been built; run the Foxy colcon build first." >&2
  exit 2
fi
if [[ "$vehicle_dry_run" != true && "$vehicle_dry_run" != false ]]; then
  echo "VEHICLE_DRY_RUN must be true or false." >&2
  exit 2
fi
if [[ "$use_rviz" != true && "$use_rviz" != false ]]; then
  echo "USE_RVIZ must be true or false." >&2
  exit 2
fi
if [[ "$use_rviz" == true && -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set. Use USE_RVIZ=false or run from a desktop terminal." >&2
  exit 2
fi

if [[ "$vehicle_dry_run" == false ]]; then
  echo "LIVE CAN MODE: commands will be sent on $can_interface."
else
  echo "DRY-RUN MODE: CAN frames are logged but not sent."
fi
echo "Starting Foxy real-vehicle navigation (map=$map_file, RViz=$use_rviz)"
echo "Press Ctrl-C to stop Nav2, safety gate, vehicle interface, RViz, and Docker."

FOXY_CONTAINER_NAME=forklift-foxy-real \
  ./scripts/foxy_docker_run.sh bash -lc '
set -euo pipefail
set +u
source /opt/ros/foxy/setup.bash
source /workspace/install_foxy/setup.bash
set -u
exec ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py \
  map:="$1" \
  nav2_params_file:="$2" \
  vehicle_dry_run:="$3" \
  can_interface:="$4" \
  use_rviz:="$5"
' _ "$map_file" "$params_file" "$vehicle_dry_run" "$can_interface" "$use_rviz"
