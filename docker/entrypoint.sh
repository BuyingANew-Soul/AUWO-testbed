#!/bin/bash
set -e

: "${WORKSPACE:=/workspace}"
ROS_DISTRO_SAFE="${ROS_DISTRO:-jazzy}"

# Fix colcon dirs (volumes often come in root-owned)
for d in src build install log; do
  sudo install -d -m 0775 -o "$(id -u)" -g "$(id -g)" "${WORKSPACE}/${d}" 2>/dev/null || true
  sudo chown -R "$(id -u):$(id -g)" "${WORKSPACE}/${d}" 2>/dev/null || true
done

# Do NOT touch ${WORKSPACE}/config (ro)

# Fix ROS home (volume/bind may be root-owned)
sudo install -d -m 0775 -o "$(id -u)" -g "$(id -g)" /home/robot/.ros 2>/dev/null || true
sudo chown -R "$(id -u):$(id -g)" /home/robot/.ros 2>/dev/null || true

# Fix RTAB-Map DB directory if you use it
if [ -d /data/rtabmap ] || mkdir -p /data/rtabmap 2>/dev/null; then
  sudo install -d -m 0775 -o "$(id -u)" -g "$(id -g)" /data/rtabmap 2>/dev/null || true
  sudo chown -R "$(id -u):$(id -g)" /data/rtabmap 2>/dev/null || true
fi

# Source ROS 2
if [ -r "/opt/ros/${ROS_DISTRO_SAFE}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO_SAFE}/setup.bash"
fi
if [ -r "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"

