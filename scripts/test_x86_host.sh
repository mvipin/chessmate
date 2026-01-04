#!/bin/bash
# ChessMate x86 Host Testing Script
# 
# This script tests x86 host components: visualization, GUI tools, and distributed communication.
# Designed for development hosts with GUI capabilities.

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
    if [ -f "install_x86/setup.bash" ]; then
        INSTALL_DIR="install_x86"
        print_status $GREEN "✅ x86 workspace found"
    elif [ -f "install/setup.bash" ]; then
        INSTALL_DIR="install"
        print_status $GREEN "✅ x86 workspace found"
    else
        print_status $RED "❌ x86 workspace not built. Please run './setup_and_build.sh' first."
        exit 1
    fi
    
    # Source workspace
    source "$INSTALL_DIR/setup.bash"
    
    # Check GUI environment
    if [ -n "$DISPLAY" ]; then
        print_status $GREEN "✅ GUI environment available"
    else
        print_status $YELLOW "⚠️  No GUI environment detected - some tests may fail"
    fi
    
    # Check ROS2 packages
    if ros2 pkg list 2>/dev/null | grep -q chessmate; then
        local pkg_count=$(ros2 pkg list 2>/dev/null | grep chessmate | wc -l)
        print_status $GREEN "✅ ChessMate packages available ($pkg_count packages)"
    else
        print_status $RED "❌ ChessMate packages not found"
        exit 1
    fi
}

# Function to test visualization
test_visualization() {
    print_header "Visualization Test"
    
    print_status $BLUE "Testing RViz2 and robot visualization..."
    
    # Check if RViz2 is available
    if command -v rviz2 &> /dev/null; then
        print_status $GREEN "✅ RViz2 available"
    else
        print_status $RED "❌ RViz2 not found"
        return 1
    fi
    
    # Check robot description package
    if ros2 pkg list 2>/dev/null | grep -q chessmate_description; then
        print_status $GREEN "✅ Robot description package available"
    else
        print_status $RED "❌ Robot description package not found"
        return 1
    fi
    
    # Test robot state publisher (brief test)
    print_status $BLUE "Testing robot state publisher..."
    timeout 15 ros2 launch chessmate_description display.launch.py &
    local launch_pid=$!
    
    sleep 10  # Let RViz start
    
    # Check if nodes are running
    if ros2 node list 2>/dev/null | grep -q robot_state_publisher; then
        print_status $GREEN "✅ Robot state publisher started successfully"
    else
        print_status $RED "❌ Robot state publisher failed to start"
        kill $launch_pid 2>/dev/null || true
        return 1
    fi
    
    # Clean shutdown
    kill $launch_pid 2>/dev/null || true
    sleep 2
    
    print_status $GREEN "✅ Visualization test passed"
}

# Function to test GUI tools
test_gui_tools() {
    print_header "GUI Tools Test"
    
    print_status $BLUE "Testing ROS2 GUI tools..."
    
    # Check rqt
    if command -v rqt &> /dev/null; then
        print_status $GREEN "✅ rqt available"
    else
        print_status $YELLOW "⚠️  rqt not found"
    fi
    
    # Check rqt_graph
    if command -v rqt_graph &> /dev/null; then
        print_status $GREEN "✅ rqt_graph available"
    else
        print_status $YELLOW "⚠️  rqt_graph not found"
    fi
    
    # Check joint_state_publisher_gui
    if ros2 pkg list 2>/dev/null | grep -q joint_state_publisher_gui; then
        print_status $GREEN "✅ joint_state_publisher_gui available"
    else
        print_status $YELLOW "⚠️  joint_state_publisher_gui not found"
    fi
    
    print_status $GREEN "✅ GUI tools test completed"
}

# Function to test distributed communication
test_distributed() {
    local pi_hostname=${1:-"chessmate-pi.local"}
    
    print_header "Distributed Communication Test"
    
    print_status $BLUE "Testing distributed ROS2 communication..."
    print_status $BLUE "Pi hostname: $pi_hostname"
    
    # Set ROS domain for distributed testing
    export ROS_DOMAIN_ID=42
    
    # Test network connectivity to Pi
    if ping -c 1 "$pi_hostname" &> /dev/null; then
        print_status $GREEN "✅ Network connectivity to Pi"
    else
        print_status $YELLOW "⚠️  Cannot reach Pi at $pi_hostname"
        print_status $BLUE "Distributed testing requires Pi to be running and accessible"
    fi
    
    # Test ROS2 discovery
    print_status $BLUE "Testing ROS2 node discovery..."
    timeout 10 ros2 node list &> /dev/null
    if [ $? -eq 0 ]; then
        print_status $GREEN "✅ ROS2 discovery working"
    else
        print_status $YELLOW "⚠️  ROS2 discovery timeout"
    fi
    
    print_status $GREEN "✅ Distributed communication test completed"
}

# Function to test all components
test_all() {
    local pi_hostname=${1:-"chessmate-pi.local"}
    
    print_header "Complete x86 Host Test"
    
    local overall_success=true
    
    # Test visualization
    if ! test_visualization; then
        overall_success=false
    fi
    
    echo ""
    
    # Test GUI tools
    if ! test_gui_tools; then
        overall_success=false
    fi
    
    echo ""
    
    # Test distributed communication
    if ! test_distributed "$pi_hostname"; then
        overall_success=false
    fi
    
    # Generate summary
    print_header "x86 Host Test Results"
    
    if [ "$overall_success" = true ]; then
        print_status $GREEN "✅ All x86 host tests PASSED"
        print_status $GREEN "🎉 x86 host system is ready for distributed testing!"
        return 0
    else
        print_status $RED "❌ Some x86 host tests FAILED"
        print_status $YELLOW "Check the output above for error details"
        return 1
    fi
}

# Function to show help
show_help() {
    echo "ChessMate x86 Host Testing Script"
    echo ""
    echo "Tests x86 host components: visualization, GUI tools, and distributed communication."
    echo ""
    echo "Usage: $0 <COMPONENT> [OPTIONS]"
    echo ""
    echo "Components:"
    echo "  visualization     Test RViz2 and robot visualization"
    echo "  gui               Test GUI tools (rqt, etc.)"
    echo "  distributed       Test distributed communication with Pi"
    echo "  all               Test all components (default)"
    echo ""
    echo "Options:"
    echo "  --pi-hostname HOST    Pi hostname for distributed testing (default: chessmate-pi.local)"
    echo ""
    echo "Examples:"
    echo "  $0 all                                    # Test all components"
    echo "  $0 visualization                          # Test visualization only"
    echo "  $0 distributed --pi-hostname 192.168.1.100    # Test distributed with specific Pi IP"
    echo ""
    echo "What this tests:"
    echo "  • RViz2 robot visualization"
    echo "  • ROS2 GUI tools availability"
    echo "  • Network connectivity to Pi"
    echo "  • Distributed ROS2 communication setup"
}

# Parse command line arguments
parse_args() {
    COMPONENT="all"
    PI_HOSTNAME="chessmate-pi.local"

    while [[ $# -gt 0 ]]; do
        case $1 in
            visualization|viz|gui|distributed|dist|all)
                COMPONENT="$1"
                shift
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
    local pi_hostname="$PI_HOSTNAME"
    
    print_header "ChessMate x86 Host Test"
    print_status $BLUE "Component: $component"
    if [ "$component" = "distributed" ] || [ "$component" = "all" ]; then
        print_status $BLUE "Pi Hostname: $pi_hostname"
    fi
    
    # Check prerequisites
    check_prerequisites
    
    case $component in
        "visualization"|"viz")
            test_visualization
            ;;
        "gui")
            test_gui_tools
            ;;
        "distributed"|"dist")
            test_distributed "$pi_hostname"
            ;;
        "all")
            test_all "$pi_hostname"
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
