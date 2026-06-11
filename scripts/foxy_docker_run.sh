#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p .foxy_home

tty_args=()
if [ -t 0 ]; then
  tty_args=(-it)
fi

docker run --rm "${tty_args[@]}" \
  --net=host \
  --user "$(id -u):$(id -g)" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e HOME=/workspace/.foxy_home \
  -e DISPLAY="${DISPLAY:-}" \
  -e QT_X11_NO_MITSHM=1 \
  -v "$PWD:/workspace" \
  -w /workspace \
  forklift-nav2:foxy \
  "$@"
