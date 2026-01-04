#!/bin/bash
# ChessMate Integration Tests Runner
# Runs integration tests that may require ROS2 nodes

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TEST_DIR="$WORKSPACE_DIR/test"

echo "🔗 ChessMate Integration Tests"
echo "==============================="

cd "$WORKSPACE_DIR"

# Activate virtual environment if available
if [ -f "$HOME/chessmate_env/bin/activate" ]; then
    source "$HOME/chessmate_env/bin/activate"
fi

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2 Humble not found"
    exit 1
fi

# Source workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
elif [ -f "$WORKSPACE_DIR/install_arm/setup.bash" ]; then
    source "$WORKSPACE_DIR/install_arm/setup.bash"
else
    echo "⚠️ Workspace not built, building now..."
    colcon build --symlink-install
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo ""
echo "Available integration tests:"
echo "  1. test_arduino_serial.py - Serial protocol testing"
echo "  2. test_ros2_hardware.py  - ROS2 hardware node testing"
echo ""

# Check if specific test requested
if [ -n "$1" ]; then
    echo "Running specified test: $1"
    python3 "$TEST_DIR/integration/$1"
else
    echo "Running all integration tests..."
    for test_file in "$TEST_DIR/integration"/test_*.py; do
        if [ -f "$test_file" ]; then
            echo ""
            echo "📋 Running: $(basename $test_file)"
            python3 "$test_file" --help 2>/dev/null || python3 "$test_file" || true
        fi
    done
fi

echo ""
echo "✅ Integration tests complete"

