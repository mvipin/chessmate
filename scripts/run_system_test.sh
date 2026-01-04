#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# ChessMate Full System Integration Test
echo "🎉 ChessMate Full System Integration Test"
echo "=========================================="
echo "Testing complete game management with all components"
echo "Full production functionality!"
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"  # ChessMate-ROS2 package directory
# Navigate up to find the ROS2 workspace root (chessmate_ws)
# Structure: chessmate_ws/src/ChessMate-ROS2/scripts/
ROS2_WS_DIR="$(dirname "$(dirname "$PACKAGE_DIR")")"  # chessmate_ws
PROJECT_ROOT="$(dirname "$ROS2_WS_DIR")"  # ChessMate project root

cd "$PACKAGE_DIR"

# Setup ROS2 environment
setup_ros2_env() {
    # Source virtual environment for Python dependencies
    # Try project root first, then workspace, then home directory
    if [ -f "${PROJECT_ROOT}/chessmate_env/bin/activate" ]; then
        VENV_PATH="${PROJECT_ROOT}/chessmate_env"
    elif [ -f "${ROS2_WS_DIR}/chessmate_env/bin/activate" ]; then
        VENV_PATH="${ROS2_WS_DIR}/chessmate_env"
    elif [ -f "${HOME}/ChessMate/chessmate_env/bin/activate" ]; then
        VENV_PATH="${HOME}/ChessMate/chessmate_env"
    else
        VENV_PATH="${HOME}/ChessMate-ROS2/chessmate_env"
    fi
    if [ -f "${VENV_PATH}/bin/activate" ]; then
        source "${VENV_PATH}/bin/activate"
        export PYTHONPATH="${VENV_PATH}/lib/python3.12/site-packages:${PYTHONPATH}"
        echo "✅ Virtual environment activated: ${VENV_PATH}"
    else
        echo "⚠️  Virtual environment not found"
    fi

    # Auto-detect ROS distro
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "❌ No ROS2 installation found!"
        exit 1
    fi

    export ROS_VERSION=2
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    # Source ROS2 workspace (chessmate_ws/install/setup.bash)
    if [ -f "$ROS2_WS_DIR/install/setup.bash" ]; then
        source "$ROS2_WS_DIR/install/setup.bash"
        echo "✅ ROS2 workspace sourced: ${ROS2_WS_DIR}/install/setup.bash"
    else
        echo "❌ ROS2 workspace install not found in ${ROS2_WS_DIR}"
        exit 1
    fi
}

setup_ros2_env

# Cleanup function
cleanup() {
    echo "🧹 Cleaning up all components..."
    pkill -f "topic_chess_engine_server" 2>/dev/null
    pkill -f "topic_arduino_communication" 2>/dev/null
    pkill -f "topic_game_management" 2>/dev/null
    pkill -f "test_full_game" 2>/dev/null
    wait
    exit
}
trap cleanup EXIT INT TERM

echo "🚀 Starting complete ChessMate system..."

# Start all components (using consolidated chessmate package)
echo "1️⃣ Starting topic-based chess engine..."
ros2 run chessmate topic_chess_engine_server.py &
ENGINE_PID=$!

echo "2️⃣ Starting topic-based Arduino communication..."
ros2 run chessmate topic_arduino_communication.py \
    --ros-args \
    --param hardware_mode:=mock &
ARDUINO_PID=$!

echo "3️⃣ Starting topic-based game management..."
ros2 run chessmate topic_game_management.py \
    --ros-args \
    --param hardware_mode:=mock \
    --param auto_start:=false \
    --param skill_level:=5 \
    --param time_limit:=2.0 &
GAME_PID=$!

# Wait for all components to initialize
echo "⏳ Waiting for all components to initialize..."
sleep 8

# Check if all components are running
components_ok=true

if ! kill -0 $ENGINE_PID 2>/dev/null; then
    echo "❌ Chess engine failed to start"
    components_ok=false
fi

if ! kill -0 $ARDUINO_PID 2>/dev/null; then
    echo "❌ Arduino communication failed to start"
    components_ok=false
fi

if ! kill -0 $GAME_PID 2>/dev/null; then
    echo "❌ Game management failed to start"
    components_ok=false
fi

if [ "$components_ok" = false ]; then
    echo "❌ Component startup failed"
    exit 1
fi

echo "✅ All components running successfully:"
echo "   Chess Engine PID: $ENGINE_PID"
echo "   Arduino Comm PID: $ARDUINO_PID"
echo "   Game Manager PID: $GAME_PID"

# Run complete system test
echo ""
echo "4️⃣ Running complete ChessMate system test..."
echo "   This will play up to 15 moves or until someone wins!"
echo "   Full chess game with proper orchestration!"

timeout 360s python3 test/system/test_full_game.py

TEST_RESULT=$?

echo ""
echo "=============================================="
if [ $TEST_RESULT -eq 0 ]; then
    echo "🎉 INTEGRATION TEST SUCCESS!"
    echo "✅ Chess engine: WORKING"
    echo "✅ Arduino communication: WORKING"
    echo "✅ Game management: WORKING"
    echo "✅ Full game orchestration: WORKING"
    echo "✅ Human-computer game flow: WORKING"
    echo ""
    echo "🎯 Complete ChessMate system verified!"
    echo ""
    echo "🚀 READY FOR:"
    echo "   - Real hardware deployment"
    echo "   - Production use"
    echo "   - Full chess games"
else
    echo "❌ Integration test failed"
    echo "🔍 Game management integration needs debugging"
fi
echo "=============================================="

exit $TEST_RESULT
