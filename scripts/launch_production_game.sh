#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# Launch Production ChessMate Game
echo "🎮 ChessMate Production Game Launch"
echo "=================================="

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

# Setup ROS2 environment
setup_ros2_env() {
    # Source virtual environment for Python dependencies
    # Try workspace-relative path first, then fall back to home directory
    if [ -f "${WORKSPACE_DIR}/chessmate_env/bin/activate" ]; then
        VENV_PATH="${WORKSPACE_DIR}/chessmate_env"
    else
        VENV_PATH="${HOME}/ChessMate-ROS2/chessmate_env"
    fi
    if [ -f "${VENV_PATH}/bin/activate" ]; then
        source "${VENV_PATH}/bin/activate"
        export PYTHONPATH="${VENV_PATH}/lib/python3.12/site-packages:${PYTHONPATH}"
        echo "✅ Virtual environment activated"
    else
        echo "⚠️  Virtual environment not found at ${VENV_PATH}"
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

    # Source workspace
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    fi
    echo "✅ ROS2 environment ready"
}

# Parse command line arguments
HARDWARE_MODE="real"  # Default to real mode

while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            HARDWARE_MODE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--mode <mock|real>]"
            echo ""
            echo "Options:"
            echo "  --mode <mock|real>   Set hardware mode (default: real)"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                   # Run in real mode (default)"
            echo "  $0 --mode real       # Run with real hardware"
            echo "  $0 --mode mock       # Run in mock mode (no sensors)"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate mode parameter
if [[ "$HARDWARE_MODE" != "mock" && "$HARDWARE_MODE" != "real" ]]; then
    echo "❌ Error: Invalid mode '$HARDWARE_MODE'. Must be 'mock' or 'real'"
    exit 1
fi

echo "Hardware mode: $HARDWARE_MODE"
echo ""

# Setup ROS2 environment
setup_ros2_env

# Check for controller availability (using udev symlinks)
echo "🔍 Checking for Pico controllers..."

CHESSBOARD_PORT="/dev/chessboard"
ROBOT_PORT="/dev/robot"

# Fallback to ttyACM* if udev symlinks not found
if [ ! -e "$CHESSBOARD_PORT" ]; then
    if [ -e "/dev/ttyACM0" ]; then
        CHESSBOARD_PORT="/dev/ttyACM0"
    fi
fi

if [ ! -e "$ROBOT_PORT" ]; then
    if [ -e "/dev/ttyACM1" ]; then
        ROBOT_PORT="/dev/ttyACM1"
    fi
fi

if [ ! -e "$CHESSBOARD_PORT" ] || [ ! -e "$ROBOT_PORT" ]; then
    echo "❌ Error: Pico controllers not found:"
    [ ! -e "$CHESSBOARD_PORT" ] && echo "   ChessBoard controller missing"
    [ ! -e "$ROBOT_PORT" ] && echo "   Robot controller missing"
    echo ""
    echo "💡 Please connect the Pico controllers and try again"
    exit 1
else
    echo "✅ Pico controllers found:"
    echo "   ChessBoard controller: $CHESSBOARD_PORT"
    echo "   Robot controller: $ROBOT_PORT"
    echo "   Mode: $HARDWARE_MODE"
fi

# Cleanup function
cleanup() {
    echo "🧹 Cleaning up production components..."
    pkill -f "chess_engine_server" 2>/dev/null
    pkill -f "arduino_communication" 2>/dev/null
    pkill -f "game_management" 2>/dev/null
    wait
    exit
}
trap cleanup EXIT INT TERM

echo "🚀 Starting ChessMate production system..."

# Start chess engine
echo "1️⃣ Starting Stockfish chess engine..."
ros2 run chessmate chess_engine_server &
ENGINE_PID=$!

# Start Arduino communication with real hardware
echo "2️⃣ Starting Arduino communication (${HARDWARE_MODE} mode)..."
ros2 run chessmate arduino_communication \
    --ros-args \
    --param hardware_mode:=${HARDWARE_MODE} \
    --param chessboard_port:=${CHESSBOARD_PORT} \
    --param robot_port:=${ROBOT_PORT} &
ARDUINO_PID=$!

# Start game management
echo "3️⃣ Starting game management..."
ros2 run chessmate game_management \
    --ros-args \
    --param hardware_mode:=${HARDWARE_MODE} \
    --param auto_start:=true \
    --param skill_level:=19 \
    --param time_limit:=3.0 &
GAME_PID=$!

# Wait for components to initialize
echo "⏳ Waiting for components to initialize..."
sleep 10

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

echo ""
echo "✅ All components running successfully:"
echo "   Chess Engine PID: $ENGINE_PID"
echo "   Arduino Comm PID: $ARDUINO_PID"
echo "   Game Manager PID: $GAME_PID"
echo ""
echo "🎮 ChessMate is ready for play!"
echo ""
echo "📋 Game Status:"
echo "   - Hardware mode: ${HARDWARE_MODE}"
echo "   - ChessBoard: ${CHESSBOARD_PORT}"
echo "   - Robot: ${ROBOT_PORT}"
echo "   - Skill level: ${SKILL_LEVEL:-19}"
echo ""
echo "🎯 To monitor the game:"
echo "   ros2 topic echo /game/state"
echo "   ros2 topic echo /chessboard/moves"
echo ""
echo "🛑 Press Ctrl+C to stop the game"
echo ""

# Keep running until interrupted
while true; do
    sleep 1
    
    # Check if any component died
    if ! kill -0 $ENGINE_PID 2>/dev/null; then
        echo "❌ Chess engine died!"
        break
    fi
    
    if ! kill -0 $ARDUINO_PID 2>/dev/null; then
        echo "❌ Arduino communication died!"
        break
    fi
    
    if ! kill -0 $GAME_PID 2>/dev/null; then
        echo "❌ Game management died!"
        break
    fi
done

echo "🏁 ChessMate production game ended"
