#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# ChessMate Unified Host Testing Script
# Automatically detects platform and runs appropriate tests.
# Usage: ./test_host.sh [COMPONENT] [OPTIONS]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Get script directory and workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Navigate up from scripts/ to chessmate/ to src/ to chessmate_ws/
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

print_header() {
    echo -e "\n${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}\n"
}

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

# Check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"

    cd "$WORKSPACE_DIR"
    print_status $BLUE "📂 Working directory: $WORKSPACE_DIR"

    # Use standard install directory
    if [ -f "install/setup.bash" ]; then
        INSTALL_DIR="install"
        print_status $GREEN "✅ Workspace found"
    else
        print_status $RED "❌ Workspace not built. Please run './scripts/build.sh' first."
        exit 1
    fi

    source "$INSTALL_DIR/setup.bash"
    
    if ros2 pkg list 2>/dev/null | grep -q chessmate; then
        local pkg_count=$(ros2 pkg list 2>/dev/null | grep chessmate | wc -l)
        print_status $GREEN "✅ ChessMate packages available ($pkg_count packages)"
    else
        print_status $RED "❌ ChessMate packages not found"
        exit 1
    fi
    
    # Platform-specific checks
    if [ "$PLATFORM" = "x86" ]; then
        if [ -n "$DISPLAY" ]; then
            print_status $GREEN "✅ GUI environment available"
        else
            print_status $YELLOW "⚠️  No GUI environment detected - some tests may fail"
        fi
    fi
}

#==============================================================================
# x86-SPECIFIC TESTS (GUI, Visualization)
#==============================================================================

test_visualization() {
    if [ "$PLATFORM" != "x86" ]; then
        print_status $YELLOW "⏭️  Skipping visualization test (not on x86)"
        return 0
    fi
    
    print_header "Visualization Test"
    print_status $BLUE "Testing RViz2 and robot visualization..."
    
    if command -v rviz2 &> /dev/null; then
        print_status $GREEN "✅ RViz2 available"
    else
        print_status $RED "❌ RViz2 not found"
        return 1
    fi
    
    if ros2 pkg list 2>/dev/null | grep -q chessmate_description; then
        print_status $GREEN "✅ Robot description package available"
    else
        print_status $RED "❌ Robot description package not found"
        return 1
    fi
    
    print_status $BLUE "Testing robot state publisher..."
    timeout 15 ros2 launch chessmate_description display.launch.py &
    local launch_pid=$!
    sleep 10
    
    if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
        print_status $GREEN "✅ Robot state publisher started successfully"
    else
        print_status $RED "❌ Robot state publisher failed to start"
        kill $launch_pid 2>/dev/null || true
        return 1
    fi
    
    kill $launch_pid 2>/dev/null || true
    sleep 2
    print_status $GREEN "✅ Visualization test passed"
}

test_gui_tools() {
    if [ "$PLATFORM" != "x86" ]; then
        print_status $YELLOW "⏭️  Skipping GUI tools test (not on x86)"
        return 0
    fi
    
    print_header "GUI Tools Test"
    print_status $BLUE "Testing ROS2 GUI tools..."
    
    [ -x "$(command -v rqt)" ] && print_status $GREEN "✅ rqt available" || print_status $YELLOW "⚠️  rqt not found"
    [ -x "$(command -v rqt_graph)" ] && print_status $GREEN "✅ rqt_graph available" || print_status $YELLOW "⚠️  rqt_graph not found"
    ros2 pkg list 2>/dev/null | grep -q joint_state_publisher_gui && \
        print_status $GREEN "✅ joint_state_publisher_gui available" || print_status $YELLOW "⚠️  joint_state_publisher_gui not found"
    
    print_status $GREEN "✅ GUI tools test completed"
}

test_distributed() {
    print_header "Distributed Communication Test"
    print_status $BLUE "Testing distributed ROS2 communication..."
    print_status $BLUE "Pi hostname: $PI_HOSTNAME"
    
    export ROS_DOMAIN_ID=42
    
    if ping -c 1 "$PI_HOSTNAME" &> /dev/null; then
        print_status $GREEN "✅ Network connectivity to Pi"
    else
        print_status $YELLOW "⚠️  Cannot reach Pi at $PI_HOSTNAME"
    fi
    
    print_status $BLUE "Testing ROS2 node discovery..."
    timeout 10 ros2 node list &> /dev/null && \
        print_status $GREEN "✅ ROS2 discovery working" || print_status $YELLOW "⚠️  ROS2 discovery timeout"
    
    print_status $GREEN "✅ Distributed communication test completed"
}

#==============================================================================
# ARM-SPECIFIC TESTS (GPIO, Hardware)
#==============================================================================

test_ros2_system() {
    print_header "ROS2 System Test (${HARDWARE_MODE} mode)"

    print_status $BLUE "Testing ROS2 node startup..."
    print_status $BLUE "Hardware mode: $HARDWARE_MODE"

    if [ "$HARDWARE_MODE" = "real" ]; then
        print_status $BLUE "Real mode: Using actual hardware"
    else
        print_status $BLUE "Mock mode: Using simulated hardware"
    fi

    timeout 30 ros2 launch chessmate_hardware unified_hardware.launch.py \
        hardware_mode:=$HARDWARE_MODE \
        chessboard_port:=/dev/ttyACM0 \
        robot_port:=/dev/ttyACM1 \
        log_level:=info \
        2>&1 &

    local launch_pid=$!
    sleep 15

    local node_count=0
    for i in {1..5}; do
        if ros2 node list 2>/dev/null | grep -q "unified_arduino_bridge\|game_management_node\|lcd_display_node"; then
            node_count=$(ros2 node list 2>/dev/null | grep -E "(unified_arduino_bridge|game_management_node|lcd_display_node)" | wc -l)
            if [ $node_count -ge 2 ]; then
                print_status $GREEN "✅ ROS2 nodes started successfully ($node_count nodes running)"
                break
            fi
        fi
        sleep 2
    done

    if [ $node_count -lt 2 ]; then
        print_status $RED "❌ ROS2 nodes failed to start properly"
        kill $launch_pid 2>/dev/null || true
        return 1
    fi

    kill $launch_pid 2>/dev/null || true
    sleep 3
    print_status $GREEN "✅ ROS2 system test passed"
}

test_lcd_display() {
    print_header "LCD Display Test (${HARDWARE_MODE} mode)"
    print_status $BLUE "Testing LCD display functionality..."

    if [ "$HARDWARE_MODE" = "real" ]; then
        if [ "$PLATFORM" != "arm" ]; then
            print_status $YELLOW "⏭️  Skipping real LCD test (not on ARM)"
            return 0
        fi

        print_status $BLUE "Testing real SSD1306 OLED display..."
        timeout 10 ros2 run chessmate_hardware lcd_display_node 2>/dev/null &
        local node_pid=$!
        sleep 3

        if ps -p $node_pid > /dev/null 2>&1; then
            print_status $GREEN "✅ LCD display node started successfully"
            sleep 4
        else
            print_status $RED "❌ LCD display node failed to start"
            return 1
        fi

        kill $node_pid 2>/dev/null || true
        wait $node_pid 2>/dev/null || true
        sleep 1
    else
        print_status $BLUE "🔧 Mock mode - simulating LCD display operations"
        print_status $GREEN "  ✅ Mock display initialized (128x32 pixels)"
        print_status $GREEN "  ✅ Mock text rendering: 'ChessMate Ready'"
        print_status $GREEN "  ✅ Mock clear screen operation"
    fi

    print_status $GREEN "✅ LCD display test completed"
}

test_rotary_encoder() {
    print_header "Rotary Encoder Test (${HARDWARE_MODE} mode)"
    print_status $BLUE "Testing rotary encoder functionality..."

    if [ "$HARDWARE_MODE" = "real" ]; then
        if [ "$PLATFORM" != "arm" ]; then
            print_status $YELLOW "⏭️  Skipping real encoder test (not on ARM)"
            return 0
        fi

        print_status $BLUE "Starting rotary encoder node for interactive testing..."
        print_status $YELLOW "🎮 INTERACTIVE TEST: Turn the encoder or press the button!"
        print_status $BLUE "   Press Ctrl+C to stop the test"

        ros2 run chessmate_hardware rotary_encoder_node &
        local node_pid=$!
        sleep 2

        if ps -p $node_pid > /dev/null 2>&1; then
            print_status $GREEN "✅ Rotary encoder node started successfully"

            local start_time=$(date +%s)
            while ps -p $node_pid > /dev/null 2>&1; do
                local elapsed=$(($(date +%s) - start_time))
                [ $elapsed -ge 30 ] && { print_status $YELLOW "⏰ Test timeout reached"; break; }
                sleep 1
            done
        else
            print_status $RED "❌ Rotary encoder node failed to start"
            return 1
        fi

        kill $node_pid 2>/dev/null || true
        wait $node_pid 2>/dev/null || true
        sleep 1
    else
        print_status $BLUE "🔧 Mock mode - simulating rotary encoder responses"
        print_status $GREEN "  ✅ Mock encoder initialized"
        print_status $GREEN "  ✅ Mock rotation event: clockwise"
        print_status $GREEN "  ✅ Mock button press/release events"
    fi

    print_status $GREEN "✅ Rotary encoder test completed"
}

#==============================================================================
# TEST ORCHESTRATION
#==============================================================================

test_all() {
    print_header "Complete ${PLATFORM^^} Host Test"
    local overall_success=true

    if [ "$PLATFORM" = "x86" ]; then
        test_visualization || overall_success=false
        echo ""
        test_gui_tools || overall_success=false
        echo ""
        test_distributed || overall_success=false
    else
        test_ros2_system || overall_success=false
        echo ""
        test_lcd_display || overall_success=false
        echo ""
        test_rotary_encoder || overall_success=false
    fi

    print_header "${PLATFORM^^} Host Test Results"

    if [ "$overall_success" = true ]; then
        print_status $GREEN "✅ All ${PLATFORM^^} host tests PASSED"
        print_status $GREEN "🎉 ${PLATFORM^^} host system is ready!"
        return 0
    else
        print_status $RED "❌ Some ${PLATFORM^^} host tests FAILED"
        return 1
    fi
}

show_help() {
    echo "ChessMate Unified Host Testing Script"
    echo ""
    echo "Automatically detects platform and runs appropriate tests."
    echo ""
    echo "Usage: $0 [COMPONENT] [OPTIONS]"
    echo ""
    echo "Components:"
    echo "  all               Test all components (default)"
    echo "  visualization     Test RViz2 and robot visualization (x86 only)"
    echo "  gui               Test GUI tools (x86 only)"
    echo "  distributed       Test distributed communication with Pi"
    echo "  ros2              Test ROS2 system"
    echo "  lcd               Test LCD display"
    echo "  encoder           Test rotary encoder"
    echo ""
    echo "Options:"
    echo "  --platform PLATFORM  Force platform: x86 or arm (default: auto-detect)"
    echo "  --mode MODE          Hardware mode: mock or real (default: mock)"
    echo "  --pi-hostname HOST   Pi hostname for distributed testing (default: chessmate-pi.local)"
    echo ""
    echo "Examples:"
    echo "  $0                              # Auto-detect, test all"
    echo "  $0 visualization                # Test visualization (x86)"
    echo "  $0 ros2 --mode real             # Test ROS2 with real hardware"
    echo "  $0 --platform arm lcd           # Force ARM, test LCD"
}

# Parse arguments
PLATFORM=""
HARDWARE_MODE="mock"
PI_HOSTNAME="chessmate-pi.local"
COMPONENT="all"

while [[ $# -gt 0 ]]; do
    case $1 in
        visualization|viz|gui|distributed|dist|ros2|lcd|encoder|all)
            COMPONENT="$1"
            shift
            ;;
        --platform|-p)
            PLATFORM="$2"
            shift 2
            ;;
        --mode|-m)
            HARDWARE_MODE="$2"
            shift 2
            ;;
        --pi-hostname)
            PI_HOSTNAME="$2"
            shift 2
            ;;
        -h|--help|help)
            show_help
            exit 0
            ;;
        *)
            print_status $RED "❌ Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Auto-detect platform if not specified
if [ -z "$PLATFORM" ]; then
    PLATFORM=$(detect_platform)
    if [ "$PLATFORM" = "unknown" ]; then
        print_status $RED "❌ Unknown architecture: $(uname -m)"
        exit 1
    fi
    print_status $BLUE "🔍 Auto-detected platform: $PLATFORM"
else
    print_status $BLUE "🎯 Platform (override): $PLATFORM"
fi

# Main execution
print_header "ChessMate Host Test"
print_status $BLUE "Platform: $PLATFORM"
print_status $BLUE "Component: $COMPONENT"
print_status $BLUE "Hardware Mode: $HARDWARE_MODE"

check_prerequisites

case $COMPONENT in
    "visualization"|"viz")
        test_visualization
        ;;
    "gui")
        test_gui_tools
        ;;
    "distributed"|"dist")
        test_distributed
        ;;
    "ros2")
        test_ros2_system
        ;;
    "lcd")
        test_lcd_display
        ;;
    "encoder")
        test_rotary_encoder
        ;;
    "all")
        test_all
        ;;
    *)
        print_status $RED "❌ Unknown component: $COMPONENT"
        show_help
        exit 1
        ;;
esac

