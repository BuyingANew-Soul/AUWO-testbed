#!/bin/bash
set -e

: "${WORKSPACE:=/workspace}"
ROS_DISTRO_SAFE="${ROS_DISTRO:-jazzy}"

# --- Create writable dirs (ok if they already exist) ---
for d in src build install log; do
  sudo mkdir -p "${WORKSPACE}/${d}"
done

# --- Chown only if directory is writable; skip read-only mounts (e.g., config) ---
for d in src build install log; do
  if [ -w "${WORKSPACE}/${d}" ]; then
    sudo chown -R robot:robot "${WORKSPACE}/${d}" || true
  fi
done

# (Do NOT chown ${WORKSPACE}/config — it may be mounted :ro)

# --- Source ROS 2 ---
if [ -r "/opt/ros/${ROS_DISTRO_SAFE}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_SAFE}/setup.bash"
fi
if [ -r "${WORKSPACE}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
