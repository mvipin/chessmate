# Copyright (c) 2025 Vipin M
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.8)
project(chessmate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============ FIND DEPENDENCIES ============
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ============ MESSAGE/SERVICE GENERATION ============
# Note: ChessMove.msg must come before GameState.msg (dependency)
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages (ordered by dependencies)
  "msg/ChessMove.msg"
  "msg/ChessPiece.msg"
  "msg/BoardState.msg"
  "msg/GameState.msg"
  "msg/GripperCommand.msg"
  "msg/JointCommand.msg"
  "msg/RobotStatus.msg"
  "msg/RotaryEncoderEvent.msg"
  "msg/LCDCommand.msg"
  "msg/ArduinoCommand.msg"
  "msg/RobotAnimation.msg"
  "msg/SensorReading.msg"
  # Services
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  "srv/ChessSquareKinematics.srv"
  "srv/CalculateMove.srv"
  "srv/GetBestMove.srv"
  "srv/EvaluatePosition.srv"
  "srv/ValidateMove.srv"
  "srv/UpdateGameState.srv"
  "srv/CalibrateArm.srv"
  "srv/ExecuteMove.srv"
  "srv/SetBoardMode.srv"
  # Actions
  "action/ExecuteChessMove.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs builtin_interfaces
)

# ============ PYTHON PACKAGE INSTALLATION ============
# Get Python version for correct installation path
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

# Install Python modules to the same location as rosidl-generated modules
# Using install(DIRECTORY) instead of ament_python_install_package to avoid
# target name conflict with rosidl_generate_interfaces
install(DIRECTORY chessmate/
  DESTINATION ${PYTHON_INSTALL_DIR}/chessmate
  FILES_MATCHING PATTERN "*.py"
)

# ============ EXECUTABLE NODES ============
# Install Python executable scripts
install(PROGRAMS
  # Engine nodes
  chessmate/engine/topic_chess_engine_server.py
  chessmate/engine/topic_game_management.py
  chessmate/engine/chess_engine_server.py
  chessmate/engine/chess_game_manager.py
  chessmate/engine/simple_chess_engine.py
  # Hardware nodes
  chessmate/hardware/topic_arduino_communication.py
  chessmate/hardware/arduino_communication_node.py
  chessmate/hardware/unified_arduino_bridge.py
  chessmate/hardware/rotary_encoder_node.py
  chessmate/hardware/lcd_display_node.py
  chessmate/hardware/robot_animation_controller.py
  chessmate/hardware/game_management_node.py
  chessmate/hardware/unified_hardware_test.py
  # Scripts
  scripts/kinematics_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============ DESCRIPTION FILES ============
# Install URDF files
install(DIRECTORY description/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(DIRECTORY description/meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install RViz configurations
install(DIRECTORY description/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ============ CONFIGURATION FILES ============
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============ LAUNCH FILES ============
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# ============ RESOURCE MARKER ============
install(FILES resource/chessmate
  DESTINATION share/ament_index/resource_index/packages
)

# ============ TESTING ============
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # Linting
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # Python tests
  ament_add_pytest_test(test_scara_kinematics test/unit/test_scara_kinematics.py)
endif()

# ============ EXPORT ============
ament_export_dependencies(rosidl_default_runtime)
ament_package()

