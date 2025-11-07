#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."

echo "Starting excavator container..."
export USER_ID="$(id -u)"
export GROUP_ID="$(id -g)"
export WORKSPACE="${WORKSPACE:-/workspace}"
export DISPLAY="${DISPLAY:-}"

# Allow X11 forwarding (safe enough for local dev)
xhost +local:root >/dev/null 2>&1 || true

docker compose -f docker/docker-compose.yml up -d

echo "Container started. Attach with:"
echo "  docker exec -it auwo_dev bash"
