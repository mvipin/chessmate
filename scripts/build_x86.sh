#!/bin/bash
# Build ChessMate for x86 (Linux Host) Architecture
# Usage: ./build_x86.sh [package_name]

set -e

# Change to workspace directory (parent of scripts directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

echo "🏗️  Building ChessMate for x86 Architecture"
echo "============================================"

# Setup environment
export ROS_DOMAIN_ID=0

# Auto-detect ROS2 distribution
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "Using ROS2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "Using ROS2 Humble"
else
    echo "Error: No supported ROS2 distribution found"
    exit 1
fi

# Detect Python version dynamically
PYTHON_EXECUTABLE=$(which python3)
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "Using Python $PYTHON_VERSION at $PYTHON_EXECUTABLE"

# Build arguments with dynamic Python configuration
BUILD_ARGS=(
    --build-base build_x86
    --install-base install_x86
    --cmake-args
        -DPython3_EXECUTABLE=$PYTHON_EXECUTABLE
        -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE
        -DPython3_FIND_STRATEGY=LOCATION
        -DCMAKE_BUILD_TYPE=Release
)

# Add package selection if specified
if [ $# -gt 0 ]; then
    BUILD_ARGS+=(--packages-select "$@")
    echo "Building packages: $*"
else
    echo "Building all packages"
fi

# Set separate log directory for x86
export COLCON_LOG_PATH="$(pwd)/log_x86"
mkdir -p "$COLCON_LOG_PATH"

# Build
echo "Building..."
colcon build "${BUILD_ARGS[@]}"

echo ""
echo "✅ x86 build complete!"
echo "📁 Build artifacts in: build_x86/"
echo "📁 Install artifacts in: install_x86/"
echo "📁 Logs in: log_x86/"
echo ""
echo "🚀 To use:"
echo "   source install_x86/setup.bash"
echo "   # or run: ./source_x86.sh"
