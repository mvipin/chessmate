#!/bin/bash
# ChessMate Unit Tests Runner
# Runs pytest unit tests (no ROS2 required)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
TEST_DIR="$WORKSPACE_DIR/src/chessmate/test"

echo "🧪 ChessMate Unit Tests"
echo "========================"

cd "$WORKSPACE_DIR"

# Activate virtual environment if available
if [ -f "$HOME/chessmate_env/bin/activate" ]; then
    source "$HOME/chessmate_env/bin/activate"
fi

# Source ROS2 for message types
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace for chessmate package
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
elif [ -f "$WORKSPACE_DIR/install_arm/setup.bash" ]; then
    source "$WORKSPACE_DIR/install_arm/setup.bash"
fi

echo "Running unit tests..."
python3 -m pytest "$TEST_DIR/unit/" -v --tb=short "$@"

echo ""
echo "✅ Unit tests complete"

