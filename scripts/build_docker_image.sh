#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."

echo "Building Docker image..."
export USER_ID="$(id -u)"
export GROUP_ID="$(id -g)"
export WORKSPACE="${WORKSPACE:-/workspace}"

docker compose -f docker/docker-compose.yml build

echo "Docker image built successfully!"
