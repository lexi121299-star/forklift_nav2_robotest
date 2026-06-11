#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
docker build -t forklift-nav2:foxy -f docker/foxy/Dockerfile .
