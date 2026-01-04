#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# ChessBoard Controller Test Script
#
# Uses udev symlinks for consistent device naming:
#   /dev/chessboard - ChessBoard controller (primary)
#   /dev/ttyACM0    - Fallback if udev rules not installed

echo "🏁 ChessBoard Controller Test"
echo "============================="

# Check if Python script exists
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ ! -f "$SCRIPT_DIR/test_chessboard_controller.py" ]; then
    echo "❌ test_chessboard_controller.py not found"
    exit 1
fi

# Detect ChessBoard controller port (prefer udev symlink)
if [ -e "/dev/chessboard" ]; then
    CHESSBOARD_PORT="/dev/chessboard"
    echo "✅ Found ChessBoard controller via udev symlink: $CHESSBOARD_PORT"
elif [ -e "/dev/ttyACM0" ]; then
    CHESSBOARD_PORT="/dev/ttyACM0"
    echo "⚠️  udev symlink not found, using fallback: $CHESSBOARD_PORT"
    echo "   To enable persistent naming, install udev rules:"
    echo "   sudo cp chessmate_dev_ws/99-chessmate-pico.rules /etc/udev/rules.d/"
    echo "   sudo udevadm control --reload-rules && sudo udevadm trigger"
elif [ -e "/dev/ttyACM1" ]; then
    CHESSBOARD_PORT="/dev/ttyACM1"
    echo "⚠️  Using alternate port: $CHESSBOARD_PORT"
else
    echo "❌ ChessBoard controller not found"
    echo "   Checked: /dev/chessboard, /dev/ttyACM0, /dev/ttyACM1"
    echo "   Make sure the ChessBoard Pi Pico is connected via USB"
    exit 1
fi

# Check permissions
if [ ! -r "$CHESSBOARD_PORT" ] || [ ! -w "$CHESSBOARD_PORT" ]; then
    echo "⚠️  Permission issue with $CHESSBOARD_PORT"
    echo "   You may need to add your user to the dialout group:"
    echo "   sudo usermod -a -G dialout $USER"
    echo "   Then log out and log back in"
fi

# Run the test
echo "🚀 Starting ChessBoard controller test..."
cd "$SCRIPT_DIR"
python3 test_chessboard_controller.py "$@"
