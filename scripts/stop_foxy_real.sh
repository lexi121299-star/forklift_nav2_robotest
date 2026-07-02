#!/usr/bin/env bash
set -euo pipefail

container_name="forklift-foxy-real"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not in PATH." >&2
  exit 2
fi
if ! docker container inspect "$container_name" >/dev/null 2>&1; then
  echo "Foxy real-vehicle stack is not running; no container needs cleanup."
  exit 0
fi

echo "Stopping $container_name ..."
if ! docker stop --time 10 "$container_name" >/dev/null 2>&1; then
  echo "docker stop was denied; stopping ros2 launch inside the container ..."
  docker exec "$container_name" bash -lc '
    launch_pid=$(pgrep -f "^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch forklift_nav2_demo forklift_real_navigation.launch.py" | head -n 1)
    if [[ -n "$launch_pid" ]]; then
      kill -TERM "$launch_pid"
    fi
  ' >/dev/null 2>&1 || true

  for _ in $(seq 1 20); do
    if ! docker container inspect "$container_name" >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done
fi
if docker container inspect "$container_name" >/dev/null 2>&1; then
  docker rm -f "$container_name" >/dev/null
fi
echo "Real-vehicle stack stopped and its container was removed."
