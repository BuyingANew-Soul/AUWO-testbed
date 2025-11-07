#!/bin/bash
set -e

echo "Installing udev rules..."

# Copy rules
sudo cp config/udev/*.rules /etc/udev/rules.d/

# Reload udev
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "udev rules installed. Unplug and replug sensors."