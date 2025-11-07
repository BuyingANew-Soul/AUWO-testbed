#!/bin/bash
set -e

: "${WORKSPACE:=/workspace}"
ROS_DISTRO_SAFE="${ROS_DISTRO:-jazzy}"

# Source ROS 2 (allow unset env during sourcing)
if [ -r "/opt/ros/${ROS_DISTRO_SAFE}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_SAFE}/setup.bash"
fi

# Source overlay if built
if [ -r "${WORKSPACE}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
