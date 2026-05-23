#!/bin/bash
# Run once after cloning on a new machine to set up hardware permissions.
# Usage: bash scripts/setup_udev.sh

set -e

echo "Installing OAK-D Pro udev rules..."
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/80-movidius.rules

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Adding $USER to plugdev group..."
sudo usermod -aG plugdev $USER

echo ""
echo "Done. Unplug and replug the OAK-D Pro, then log out and back in."
