#!/bin/bash
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

# ChessMate Process Cleanup Script
#
# This script cleans up all ChessMate-related processes that may be left running
# after test failures or interrupted tests. Run this between tests to ensure
# a clean state.

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Function to kill processes by name pattern
kill_processes_by_pattern() {
    local pattern=$1
    local description=$2
    
    local pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    
    if [ -n "$pids" ]; then
        print_status $YELLOW "Found $description processes: $pids"
        echo "$pids" | xargs kill -TERM 2>/dev/null || true
        sleep 2
        
        # Force kill if still running
        local remaining_pids=$(pgrep -f "$pattern" 2>/dev/null || true)
        if [ -n "$remaining_pids" ]; then
            print_status $YELLOW "Force killing remaining processes: $remaining_pids"
            echo "$remaining_pids" | xargs kill -KILL 2>/dev/null || true
        fi
        
        print_status $GREEN "✅ Cleaned up $description processes"
    else
        print_status $BLUE "No $description processes found"
    fi
}

# Function to cleanup ROS2 processes
cleanup_ros2_processes() {
    print_header "Cleaning Up ROS2 Processes"
    
    # Kill specific ChessMate ROS2 nodes
    kill_processes_by_pattern "unified_arduino_bridge" "Unified Arduino Bridge"
    kill_processes_by_pattern "game_management_node" "Game Management"
    kill_processes_by_pattern "robot_animation_controller" "Robot Animation Controller"
    kill_processes_by_pattern "lcd_display_node" "LCD Display"
    kill_processes_by_pattern "rotary_encoder_node" "Rotary Encoder"
    kill_processes_by_pattern "chess_engine_server" "Chess Engine Server"
    kill_processes_by_pattern "stockfish_engine" "Stockfish Engine"
    
    # Kill any remaining ChessMate launch processes
    kill_processes_by_pattern "chessmate_hardware.*launch" "ChessMate Launch Files"
    kill_processes_by_pattern "integration_testing.launch" "Integration Testing Launch"
    kill_processes_by_pattern "unified_hardware.launch" "Unified Hardware Launch"
    
    # Kill rosbag recording processes
    kill_processes_by_pattern "ros2.*bag.*record" "ROS2 Bag Recording"
    kill_processes_by_pattern "rosbag.*record" "Rosbag Recording"
}

# Function to cleanup test processes
cleanup_test_processes() {
    print_header "Cleaning Up Test Processes"
    
    # Kill test scripts
    kill_processes_by_pattern "test_chessmate_system" "ChessMate System Test"
    kill_processes_by_pattern "test_pi_host" "Pi Host Test"
    kill_processes_by_pattern "test_x86_host" "x86 Host Test"
    kill_processes_by_pattern "test_rotary_encoder" "Rotary Encoder Test"
    kill_processes_by_pattern "test_lcd" "LCD Test"
    
    # Kill any Python test processes
    kill_processes_by_pattern "python.*test.*chessmate" "Python ChessMate Tests"
}

# Function to cleanup hardware processes
cleanup_hardware_processes() {
    print_header "Cleaning Up Hardware Processes"
    
    # Kill any processes using serial ports
    kill_processes_by_pattern "ttyACM" "Serial Port Processes"
    kill_processes_by_pattern "ttyUSB" "USB Serial Processes"
    
    # Kill any stuck serial communication
    kill_processes_by_pattern "cat.*tty" "Serial Cat Processes"
    kill_processes_by_pattern "echo.*tty" "Serial Echo Processes"
}

# Function to cleanup temporary files
cleanup_temp_files() {
    print_header "Cleaning Up Temporary Files"
    
    # Remove temporary rosbag files
    if [ -d "/tmp/chessmate_test_game_simulation" ]; then
        print_status $YELLOW "Removing game simulation rosbag data..."
        rm -rf /tmp/chessmate_test_game_simulation
        print_status $GREEN "✅ Removed game simulation data"
    fi
    
    if [ -d "/tmp/chessmate_test_controllers" ]; then
        print_status $YELLOW "Removing controller test rosbag data..."
        rm -rf /tmp/chessmate_test_controllers
        print_status $GREEN "✅ Removed controller test data"
    fi
    
    # Remove any other ChessMate temp files
    local temp_files=$(find /tmp -name "*chessmate*" -type f 2>/dev/null || true)
    if [ -n "$temp_files" ]; then
        print_status $YELLOW "Removing other ChessMate temp files..."
        echo "$temp_files" | xargs rm -f 2>/dev/null || true
        print_status $GREEN "✅ Removed temporary files"
    fi
}

# Function to show running processes (for verification)
show_remaining_processes() {
    print_header "Checking for Remaining ChessMate Processes"
    
    local remaining=$(pgrep -f "chessmate\|unified_arduino\|game_management\|robot_animation\|lcd_display\|rotary_encoder" 2>/dev/null || true)
    
    if [ -n "$remaining" ]; then
        print_status $YELLOW "⚠️  Some ChessMate processes may still be running:"
        ps -p $remaining -o pid,ppid,cmd 2>/dev/null || true
        echo ""
        print_status $YELLOW "You may need to manually kill these processes or reboot"
    else
        print_status $GREEN "✅ No ChessMate processes found running"
    fi
}

# Function to show help
show_help() {
    echo "ChessMate Process Cleanup Script"
    echo ""
    echo "This script cleans up all ChessMate-related processes and temporary files"
    echo "that may be left running after test failures or interrupted tests."
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --ros2-only       Clean up only ROS2 processes"
    echo "  --test-only       Clean up only test processes"
    echo "  --hardware-only   Clean up only hardware processes"
    echo "  --temp-only       Clean up only temporary files"
    echo "  --check-only      Only check for running processes (no cleanup)"
    echo "  -h, --help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                # Full cleanup (recommended)"
    echo "  $0 --ros2-only    # Clean up only ROS2 nodes"
    echo "  $0 --check-only   # Just check what's running"
    echo ""
    echo "Run this script between tests to ensure a clean state."
}

# Main execution
main() {
    local cleanup_type="full"
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --ros2-only)
                cleanup_type="ros2"
                shift
                ;;
            --test-only)
                cleanup_type="test"
                shift
                ;;
            --hardware-only)
                cleanup_type="hardware"
                shift
                ;;
            --temp-only)
                cleanup_type="temp"
                shift
                ;;
            --check-only)
                cleanup_type="check"
                shift
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
    
    print_header "ChessMate Process Cleanup"
    print_status $BLUE "Cleanup type: $cleanup_type"
    
    case $cleanup_type in
        "ros2")
            cleanup_ros2_processes
            ;;
        "test")
            cleanup_test_processes
            ;;
        "hardware")
            cleanup_hardware_processes
            ;;
        "temp")
            cleanup_temp_files
            ;;
        "check")
            show_remaining_processes
            exit 0
            ;;
        "full")
            cleanup_ros2_processes
            cleanup_test_processes
            cleanup_hardware_processes
            cleanup_temp_files
            ;;
    esac
    
    # Always show remaining processes at the end
    show_remaining_processes
    
    print_header "Cleanup Complete"
    print_status $GREEN "🎉 ChessMate process cleanup finished!"
    print_status $BLUE "System should now be in a clean state for testing."
}

# Run main function
main "$@"
