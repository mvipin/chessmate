#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# Unified ChessMate Build Script
# Automatically detects platform architecture and builds with appropriate settings.
# Usage: ./build.sh [--target x86|arm] [package_name...]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Determine workspace directory - always use ~/ChessMate/chessmate_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Navigate up from scripts/ to chessmate/ to src/ to chessmate_ws/
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$WORKSPACE_DIR"
print_status $BLUE "📂 Working directory: $WORKSPACE_DIR"

# Detect current platform architecture
detect_platform() {
    local arch=$(uname -m)
    case "$arch" in
        x86_64|amd64)
            echo "x86"
            ;;
        aarch64|arm64|armv7l|armv8l)
            echo "arm"
            ;;
        *)
            echo "unknown"
            ;;
    esac
}

# Parse command line arguments
TARGET=""
PACKAGES=()

while [[ $# -gt 0 ]]; do
    case $1 in
        --target|-t)
            TARGET="$2"
            shift 2
            ;;
        -h|--help)
            echo "ChessMate Unified Build Script"
            echo ""
            echo "Usage: $0 [OPTIONS] [package_name...]"
            echo ""
            echo "Options:"
            echo "  --target, -t TARGET   Build target: x86 or arm (default: auto-detect)"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                    # Auto-detect platform, build all packages"
            echo "  $0 chessmate          # Build only chessmate package"
            echo "  $0 --target arm       # Force ARM build on any platform"
            echo "  $0 -t x86 chessmate   # Force x86 build of chessmate package"
            exit 0
            ;;
        *)
            PACKAGES+=("$1")
            shift
            ;;
    esac
done

# Determine target platform
if [ -z "$TARGET" ]; then
    TARGET=$(detect_platform)
    if [ "$TARGET" = "unknown" ]; then
        print_status $RED "❌ Unknown architecture: $(uname -m)"
        print_status $YELLOW "Use --target to specify: x86 or arm"
        exit 1
    fi
    print_status $BLUE "🔍 Auto-detected platform: $TARGET"
else
    print_status $BLUE "🎯 Target platform (override): $TARGET"
fi

# Validate target
if [[ "$TARGET" != "x86" && "$TARGET" != "arm" ]]; then
    print_status $RED "❌ Invalid target: $TARGET"
    print_status $YELLOW "Valid targets: x86, arm"
    exit 1
fi

echo ""
echo "🏗️  Building ChessMate for ${TARGET^^} Architecture"
echo "================================================="

# Setup environment
export ROS_DOMAIN_ID=0

# Source ROS2 distribution based on target
if [ "$TARGET" = "arm" ]; then
    # ARM typically uses Humble on Raspberry Pi
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        print_status $GREEN "Using ROS2 Humble"
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
        print_status $GREEN "Using ROS2 Jazzy"
    else
        print_status $RED "❌ No supported ROS2 distribution found"
        exit 1
    fi
else
    # x86 prefers Jazzy, falls back to Humble
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
        print_status $GREEN "Using ROS2 Jazzy"
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        print_status $GREEN "Using ROS2 Humble"
    else
        print_status $RED "❌ No supported ROS2 distribution found"
        exit 1
    fi
fi

# Detect Python version dynamically
PYTHON_EXECUTABLE=$(which python3)
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
print_status $GREEN "Using Python $PYTHON_VERSION at $PYTHON_EXECUTABLE"

# Use standard colcon directories (no platform suffixes)
BUILD_DIR="build"
INSTALL_DIR="install"
LOG_DIR="log"

# Build arguments with dynamic Python configuration
BUILD_ARGS=(
    --symlink-install
    --cmake-args
        -DPython3_EXECUTABLE=$PYTHON_EXECUTABLE
        -DPYTHON_EXECUTABLE=$PYTHON_EXECUTABLE
        -DPython3_FIND_STRATEGY=LOCATION
        -DCMAKE_BUILD_TYPE=Release
)

# Add ARM-specific Python root dir
if [ "$TARGET" = "arm" ]; then
    BUILD_ARGS+=(-DPython3_ROOT_DIR=/usr)
fi

# Add package selection if specified
if [ ${#PACKAGES[@]} -gt 0 ]; then
    BUILD_ARGS+=(--packages-select "${PACKAGES[@]}")
    print_status $BLUE "Building packages: ${PACKAGES[*]}"
else
    print_status $BLUE "Building all packages"
fi

# Build
echo ""
print_status $BLUE "Building..."
colcon build "${BUILD_ARGS[@]}"

echo ""
print_status $GREEN "✅ ${TARGET^^} build complete!"
echo "📁 Build artifacts in: $BUILD_DIR/"
echo "📁 Install artifacts in: $INSTALL_DIR/"
echo "📁 Logs in: $LOG_DIR/"
echo ""
print_status $GREEN "🚀 To use:"
echo "   source $INSTALL_DIR/setup.bash"

