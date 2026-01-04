# ChessMate ROS2 Workspace

This workspace contains the ROS2 implementation of the ChessMate autonomous chess robot system. It provides hardware interfaces, game management, and testing infrastructure for both development and deployment.

## 🚀 **Quick Start**

### **Setup and Build**
```bash
# Complete workspace setup (first time)
./setup_and_build.sh

# Source the workspace
source /opt/ros/humble/setup.bash
source install_arm/setup.bash  # (or install/setup.bash on x86)
```

### **Test the System**
```bash
# Pi host component testing (ROS2, LCD, rotary encoder)
./scripts/test_pi_host.sh all mock          # Mock hardware mode
./scripts/test_pi_host.sh ros2 real         # Real hardware mode
./scripts/test_pi_host.sh lcd mock          # LCD display only

# x86 host testing (visualization, GUI tools)
./scripts/test_x86_host.sh all              # All components
./scripts/test_x86_host.sh visualization    # RViz2 only

# System integration testing (from top-level scripts/)
../scripts/test_chessmate_system.sh pico --mode mock --controller both
../scripts/test_chessmate_system.sh ros2 --mode mock --controller both
../scripts/test_chessmate_system.sh game --mode real --duration 600
```

## 📦 **ROS2 Package**

The ChessMate system uses a single consolidated package containing all components:

| Package | Purpose | Documentation |
|---------|---------|---------------|
| [`chessmate`](src/chessmate/) | Unified package with all ChessMate components | [README](src/chessmate/README.md) |

### Package Components

| Component | Location | Description |
|-----------|----------|-------------|
| Chess Engine | `chessmate/engine/` | Stockfish integration, move calculation |
| Hardware | `chessmate/hardware/` | Arduino communication, GPIO, LCD, encoder |
| Kinematics | `chessmate/kinematics/` | SCARA kinematics, coordinate mapping |
| Description | `chessmate/description/` | URDF models, meshes, RViz configs |
| Messages | `chessmate/msg/`, `srv/`, `action/` | ROS2 interface definitions |

## 🧪 **Testing Framework**

### **Phase 1: Pi Standalone Testing**
```bash
# Hardware communication tests
./scripts/test_controllers.sh

# ROS2 system integration tests  
./scripts/test_ros2_system.sh

# Complete game simulation (Pi + Controllers)
ros2 launch chessmate integration_testing.launch.py \
    test_mode:=game_simulation \
    chessboard_port:=/dev/ttyACM0 \
    robot_port:=/dev/ttyACM1
```

### **Phase 2: Distributed Testing (Pi + Host)**
**On Raspberry Pi:**
```bash
export ROS_DOMAIN_ID=42
ros2 launch chessmate pi_headless_testing.launch.py \
    test_mode:=comprehensive \
    ros_domain_id:=42
```

**On Development Host:**
```bash
export ROS_DOMAIN_ID=42
ros2 launch chessmate host_visualization.launch.py \
    pi_hostname:=chessmate-pi.local \
    ros_domain_id:=42 \
    enable_rviz:=true
```

## 🔧 **Build Commands**

### **Platform-Specific Builds**
```bash
# ARM build (Raspberry Pi)
./scripts/build_arm.sh

# x86 build (Development Host)
./scripts/build_x86.sh

# Manual build
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### **Incremental Builds**
```bash
# Rebuild the chessmate package
colcon build --packages-select chessmate --symlink-install

# Clean build
rm -rf build* install* log*
colcon build --symlink-install
```

## 🎮 **Game Simulation**

### **Complete Chess Game Simulation**
```bash
# Source workspace
source /opt/ros/humble/setup.bash
source install_arm/setup.bash

# Run complete game simulation
ros2 launch chessmate integration_testing.launch.py \
    test_mode:=game_simulation \
    chessboard_port:=/dev/ttyACM0 \
    robot_port:=/dev/ttyACM1 \
    log_level:=info
```

**What this includes:**
- ✅ ChessBoard controller communication
- ✅ Robot controller communication
- ✅ Stockfish chess engine integration
- ✅ Complete game flow simulation
- ✅ Move validation and execution
- ✅ Game state management

### **Mock vs Real Hardware**
```bash
# With real controllers (requires Pi Picos)
ros2 launch chessmate unified_hardware.launch.py \
    hardware_mode:=real \
    chessboard_port:=/dev/ttyACM0 \
    robot_port:=/dev/ttyACM1

# With mock controllers (development mode)
ros2 launch chessmate unified_hardware.launch.py \
    hardware_mode:=mock
```

## 🔍 **Troubleshooting**

### **Common Issues**

**Build Errors:**
```bash
# Check ROS2 installation
ros2 --version

# Verify dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build* install* log*
./setup_and_build.sh
```

**Controller Connection Issues:**
```bash
# Check USB devices
ls -la /dev/ttyACM*

# Check permissions
groups $USER  # Should include 'dialout'

# Test controllers manually
echo 'mode:mock' > /dev/ttyACM0 && timeout 3 cat /dev/ttyACM0
```

**ROS2 Communication Issues:**
```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list | grep chessmate

# Monitor messages
ros2 topic echo /chessmate/board_state
```

## 📊 **System Architecture**

### **Hardware Components**
- **Raspberry Pi 4**: Main computing unit
- **Pi Pico ChessBoard**: Magnetic sensor array for piece detection
- **Pi Pico Robot**: 6-DOF arm controller for piece manipulation
- **USB Serial**: Communication between Pi and Pico controllers

### **Software Components**
- **ROS2 Nodes**: Game management, hardware interfaces, chess engine
- **Launch Files**: System startup and configuration
- **Test Framework**: Automated testing and validation
- **Mock System**: Development without hardware dependencies

### **Communication Flow**
```
Pi Host (ROS2) ←→ USB Serial ←→ Pi Pico Controllers
     ↓
Chess Engine (Stockfish)
     ↓
Game Management
     ↓
Move Execution
```

## 🚀 **Development Workflow**

### **1. Setup Development Environment**
```bash
./setup_and_build.sh
source install_arm/setup.bash
```

### **2. Test Individual Components**
```bash
./scripts/test_controllers.sh      # Hardware
./scripts/test_ros2_system.sh      # ROS2 system
```

### **3. Run Complete Game Simulation**
```bash
ros2 launch chessmate integration_testing.launch.py test_mode:=game_simulation
```

### **4. Develop and Test Changes**
```bash
# Make code changes
colcon build --packages-select <package_name> --symlink-install
source install_arm/setup.bash

# Test changes
./scripts/test_ros2_system.sh
```

## 📁 **Directory Structure**

```
chessmate_dev_ws/
├── src/                    # ROS2 source packages
├── scripts/                # Build and test scripts
├── install_arm/            # ARM build output (Pi)
├── install_x86/            # x86 build output (Host)
├── build_arm/              # ARM build files
├── build_x86/              # x86 build files
├── log_arm/                # ARM build logs
├── log_x86/                # x86 build logs
└── README.md               # This file
```

## 🎯 **Next Steps**

1. **Complete Game Simulation**: Test full chess game with Pi + Controllers
2. **Host Visualization**: Set up RViz2 and GUI components
3. **Distributed Testing**: Test Pi + Host communication
4. **Hardware Integration**: Connect physical robot components
5. **Performance Optimization**: Tune timing and accuracy

For detailed package documentation, see the README files in each package directory.
For hardware setup and construction details, see the main project [README](../README.md).
