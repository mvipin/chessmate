#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# Robot Controller Test Script
#
# Uses udev symlinks for consistent device naming:
#   /dev/robot     - Robot controller (primary)
#   /dev/ttyACM1   - Fallback if udev rules not installed

echo "🤖 Robot Controller Test"
echo "========================"
echo "ℹ️  Using merged Robot.ino with USB communication support"

# Check if Python script exists
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ ! -f "$SCRIPT_DIR/test_robot_controller.py" ]; then
    echo "❌ test_robot_controller.py not found"
    exit 1
fi

# Detect Robot controller port (prefer udev symlink)
if [ -e "/dev/robot" ]; then
    ROBOT_PORT="/dev/robot"
    echo "✅ Found Robot controller via udev symlink: $ROBOT_PORT"
elif [ -e "/dev/ttyACM1" ]; then
    ROBOT_PORT="/dev/ttyACM1"
    echo "⚠️  udev symlink not found, using fallback: $ROBOT_PORT"
    echo "   To enable persistent naming, install udev rules:"
    echo "   sudo cp chessmate_dev_ws/99-chessmate-pico.rules /etc/udev/rules.d/"
    echo "   sudo udevadm control --reload-rules && sudo udevadm trigger"
elif [ -e "/dev/ttyACM0" ]; then
    ROBOT_PORT="/dev/ttyACM0"
    echo "⚠️  Using alternate port: $ROBOT_PORT"
else
    echo "❌ Robot controller not found"
    echo "   Checked: /dev/robot, /dev/ttyACM1, /dev/ttyACM0"
    echo "   Make sure the Robot Pi Pico is connected via USB"
    exit 1
fi

# Check permissions
if [ ! -r "$ROBOT_PORT" ] || [ ! -w "$ROBOT_PORT" ]; then
    echo "⚠️  Permission issue with $ROBOT_PORT"
    echo "   You may need to add your user to the dialout group:"
    echo "   sudo usermod -a -G dialout $USER"
    echo "   Then log out and log back in"
fi

# Run the test
echo "🚀 Starting Robot controller test..."
cd "$SCRIPT_DIR"
python3 test_robot_controller.py "$@"
