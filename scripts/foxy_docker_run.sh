#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p .foxy_home

tty_args=()
if [ -t 0 ]; then
  tty_args=(-it)
fi

name_args=()
if [ -n "${FOXY_CONTAINER_NAME:-}" ]; then
  name_args=(--name "$FOXY_CONTAINER_NAME")
fi

# Foxy ships CycloneDDS 0.7, which can crash while auto-selecting among the
# many host interfaces exposed by --net=host. The single-container simulation
# only needs loopback discovery. Raise the participant-index ceiling because a
# full Nav2 bringup starts more processes than Cyclone's small default range.
# Real/multi-host deployments can override CYCLONEDDS_URI before invoking this
# helper to select the vehicle network interface.
cyclonedds_uri_default='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

docker run --rm "${tty_args[@]}" "${name_args[@]}" \
  --net=host \
  --user "$(id -u):$(id -g)" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-$cyclonedds_uri_default}" \
  -e HOME=/workspace/.foxy_home \
  -e DISPLAY="${DISPLAY:-}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD:/workspace" \
  -w /workspace \
  forklift-nav2:foxy \
  "$@"
