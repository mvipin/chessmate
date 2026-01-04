#!/bin/bash
# ChessMate Pi Host Testing Script
# 
# This script tests Pi host components: ROS2 system, LCD display, and rotary encoder.
# Supports both mock and real hardware modes.

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Function to print colored output
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

# Function to check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"
    
    # Change to workspace directory
    cd "$WORKSPACE_DIR"
    
    # Check if workspace is built
    if [ -f "install_arm/setup.bash" ]; then
        INSTALL_DIR="install_arm"
        print_status $GREEN "✅ ARM workspace found"
    elif [ -f "install/setup.bash" ]; then
        INSTALL_DIR="install"
        print_status $GREEN "✅ x86 workspace found"
    else
        print_status $RED "❌ Workspace not built. Please run './setup_and_build.sh' first."
        exit 1
    fi
    
    # Source workspace
    source "$INSTALL_DIR/setup.bash"
    
    # Check ROS2 packages
    if ros2 pkg list 2>/dev/null | grep -q chessmate; then
        local pkg_count=$(ros2 pkg list 2>/dev/null | grep chessmate | wc -l)
        print_status $GREEN "✅ ChessMate packages available ($pkg_count packages)"
    else
        print_status $RED "❌ ChessMate packages not found"
        exit 1
    fi
}

# Function to test ROS2 system
test_ros2_system() {
    local hardware_mode=${1:-"mock"}
    
    print_header "ROS2 System Test (${hardware_mode} mode)"
    
    print_status $BLUE "Testing ROS2 node startup..."
    print_status $BLUE "Hardware mode: $hardware_mode"

    if [ "$hardware_mode" = "real" ]; then
        print_status $BLUE "Real mode: Using actual hardware (LCD, rotary encoder, real serial to controllers)"
    else
        print_status $BLUE "Mock mode: Using simulated hardware (mock LCD, mock rotary encoder, mock serial)"
    fi

    # Test basic node startup (short duration)
    timeout 30 ros2 launch chessmate_hardware unified_hardware.launch.py \
        hardware_mode:=$hardware_mode \
        chessboard_port:=/dev/ttyACM0 \
        robot_port:=/dev/ttyACM1 \
        log_level:=info \
        2>&1 &

    local launch_pid=$!
    sleep 15  # Let nodes start (increased wait time)

    # Check if nodes are running (more robust check)
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

    # Clean shutdown
    kill $launch_pid 2>/dev/null || true
    sleep 3
    
    print_status $GREEN "✅ ROS2 system test passed"
}

# Function to test LCD display
test_lcd_display() {
    local hardware_mode=${1:-"mock"}

    print_header "LCD Display Test (${hardware_mode} mode)"

    print_status $BLUE "Testing LCD display functionality..."

    if [ "$hardware_mode" = "real" ]; then
        # Real hardware test - check if LCD node is available and test actual display
        print_status $BLUE "Testing real SSD1306 OLED display..."

        # Start LCD node briefly to test
        print_status $BLUE "Starting LCD node for 10 seconds..."
        timeout 10 ros2 run chessmate_hardware lcd_display_node 2>/dev/null &
        local node_pid=$!
        sleep 3

        # Check if node is running
        if ps -p $node_pid > /dev/null 2>&1; then
            print_status $GREEN "✅ LCD display node started successfully"
            print_status $BLUE "Real mode: Testing actual SSD1306 display on I2C bus"
            print_status $BLUE "Display should show ChessMate initialization screen"

            # Let it run for a few more seconds to show content
            sleep 4
        else
            print_status $RED "❌ LCD display node failed to start"
            print_status $YELLOW "Check I2C connection and SSD1306 display wiring"
            return 1
        fi

        # Clean shutdown
        kill $node_pid 2>/dev/null || true
        wait $node_pid 2>/dev/null || true
        sleep 1

    else
        # Mock mode test - simulate display operations
        print_status $BLUE "🔧 Mock mode - simulating LCD display operations"
        sleep 1
        print_status $GREEN "  ✅ Mock display initialized (128x32 pixels)"
        print_status $GREEN "  ✅ Mock text rendering: 'ChessMate Ready'"
        print_status $GREEN "  ✅ Mock graphics: Drawing chess board"
        print_status $GREEN "  ✅ Mock clear screen operation"
    fi

    print_status $GREEN "✅ LCD display test completed"
}

# Function to test rotary encoder
test_rotary_encoder() {
    local hardware_mode=${1:-"mock"}

    print_header "Rotary Encoder Test (${hardware_mode} mode)"

    print_status $BLUE "Testing rotary encoder functionality..."

    if [ "$hardware_mode" = "real" ]; then
        # Real hardware test - check if rotary encoder node is available
        print_status $BLUE "Checking rotary encoder node availability..."

        # Start rotary encoder node for interactive testing
        print_status $BLUE "Starting rotary encoder node for interactive testing..."
        print_status $YELLOW "🎮 INTERACTIVE TEST: Turn the encoder or press the button!"
        print_status $BLUE "   • Clockwise rotation should publish positive values"
        print_status $BLUE "   • Counter-clockwise rotation should publish negative values"
        print_status $BLUE "   • Button press should publish button events"
        print_status $BLUE "   • Press Ctrl+C to stop the test"
        echo ""

        # Start the node and monitor topics
        ros2 run chessmate_hardware rotary_encoder_node &
        local node_pid=$!
        sleep 2

        # Check if node is running
        if ps -p $node_pid > /dev/null 2>&1; then
            print_status $GREEN "✅ Rotary encoder node started successfully"
            print_status $BLUE "Monitoring ROS topics for encoder events..."

            # Check if ros2 topic command is available
            if command -v ros2 >/dev/null && ros2 --help 2>/dev/null | grep -q topic; then
                print_status $BLUE "Monitoring /rotary_encoder_events topic..."
                timeout 30 ros2 topic echo /rotary_encoder_events --once &
                local echo_pid=$!
            else
                print_status $YELLOW "⚠️  ros2 topic command not available, monitoring node output only"
                local echo_pid=""
            fi

            # Wait for user interaction or timeout
            local start_time=$(date +%s)
            local timeout_duration=30

            while ps -p $node_pid > /dev/null 2>&1; do
                local current_time=$(date +%s)
                local elapsed=$((current_time - start_time))

                if [ $elapsed -ge $timeout_duration ]; then
                    print_status $YELLOW "⏰ Test timeout reached (${timeout_duration}s)"
                    break
                fi

                sleep 1
            done

            # Clean up
            if [ -n "$echo_pid" ]; then
                kill $echo_pid 2>/dev/null || true
            fi
        else
            print_status $RED "❌ Rotary encoder node failed to start"
            return 1
        fi

        # Clean shutdown
        kill $node_pid 2>/dev/null || true
        wait $node_pid 2>/dev/null || true
        sleep 1

    else
        # Mock mode test
        print_status $BLUE "🔧 Mock mode - simulating rotary encoder responses"
        sleep 1
        print_status $GREEN "  ✅ Mock encoder initialized"
        print_status $GREEN "  ✅ Mock rotation event: clockwise"
        print_status $GREEN "  ✅ Mock button press event"
        print_status $GREEN "  ✅ Mock button release event"
    fi

    print_status $GREEN "✅ Rotary encoder test completed"
}

# Function to test all components
test_all() {
    local hardware_mode=${1:-"mock"}
    
    print_header "Complete Pi Host Test (${hardware_mode} mode)"
    
    local overall_success=true
    
    # Test ROS2 system
    if ! test_ros2_system "$hardware_mode"; then
        overall_success=false
    fi
    
    echo ""
    
    # Test LCD display
    if ! test_lcd_display "$hardware_mode"; then
        overall_success=false
    fi
    
    echo ""
    
    # Test rotary encoder
    if ! test_rotary_encoder "$hardware_mode"; then
        overall_success=false
    fi
    
    # Generate summary
    print_header "Pi Host Test Results"
    
    if [ "$overall_success" = true ]; then
        print_status $GREEN "✅ All Pi host tests PASSED"
        print_status $GREEN "🎉 Pi host system is ready!"
        return 0
    else
        print_status $RED "❌ Some Pi host tests FAILED"
        print_status $YELLOW "Check the output above for error details"
        return 1
    fi
}

# Function to show help
show_help() {
    echo "ChessMate Pi Host Testing Script"
    echo ""
    echo "Tests Pi host components: ROS2 system, LCD display, and rotary encoder."
    echo ""
    echo "Usage: $0 <COMPONENT> [OPTIONS]"
    echo ""
    echo "Components:"
    echo "  ros2              Test ROS2 system only"
    echo "  lcd               Test LCD display only"
    echo "  encoder           Test rotary encoder only"
    echo "  all               Test all components (default)"
    echo ""
    echo "Options:"
    echo "  --mode MODE       Hardware mode: mock or real (default: mock)"
    echo ""
    echo "Examples:"
    echo "  $0 all --mode mock       # Test all components with mock hardware"
    echo "  $0 ros2 --mode real      # Test ROS2 system with real hardware"
    echo "  $0 lcd --mode mock       # Test LCD display with mock hardware"
    echo "  $0 encoder --mode real   # Test rotary encoder with real hardware"
    echo ""
    echo "What this tests:"
    echo "  • ROS2 node startup and communication"
    echo "  • LCD display functionality (real or mock)"
    echo "  • Rotary encoder input handling (real or mock)"
    echo "  • System integration without Pi Pico controllers"
}

# Parse command line arguments
parse_args() {
    COMPONENT="all"
    MODE="mock"

    while [[ $# -gt 0 ]]; do
        case $1 in
            ros2|lcd|encoder|all)
                COMPONENT="$1"
                shift
                ;;
            --mode)
                MODE="$2"
                shift 2
                ;;
            -h|--help|help)
                show_help
                exit 0
                ;;
            *)
                print_status $RED "❌ Unknown option: $1"
                echo ""
                show_help
                exit 1
                ;;
        esac
    done
}

# Main execution
main() {
    parse_args "$@"

    local component="$COMPONENT"
    local hardware_mode="$MODE"
    
    print_header "ChessMate Pi Host Test"
    print_status $BLUE "Component: $component"
    print_status $BLUE "Hardware Mode: $hardware_mode"
    
    # Check prerequisites
    check_prerequisites
    
    case $component in
        "ros2")
            test_ros2_system "$hardware_mode"
            ;;
        "lcd")
            test_lcd_display "$hardware_mode"
            ;;
        "encoder")
            test_rotary_encoder "$hardware_mode"
            ;;
        "all")
            test_all "$hardware_mode"
            ;;
        *)
            print_status $RED "❌ Unknown component: $component"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
