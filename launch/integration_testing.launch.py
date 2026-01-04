#!/usr/bin/env python3
"""
ChessMate Integration Testing Launch File

This launch file starts all required nodes for comprehensive system integration testing,
including mock hardware simulation, Stockfish engine integration, and end-to-end
chess game workflow validation.

Usage:
  # Full integration test with mock hardware
  ros2 launch chessmate_hardware integration_testing.launch.py test_mode:=full

  # Quick integration test
  ros2 launch chessmate_hardware integration_testing.launch.py test_mode:=quick

  # Controller communication test only
  ros2 launch chessmate_hardware integration_testing.launch.py test_mode:=controllers

  # End-to-end game simulation
  ros2 launch chessmate_hardware integration_testing.launch.py test_mode:=game_simulation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for integration testing"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('chessmate')
    config_file = os.path.join(pkg_dir, 'config', 'unified_hardware_config.yaml')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'test_mode',
            default_value='full',
            description='Test mode: full, quick, controllers, game_simulation'
        ),
        DeclareLaunchArgument(
            'chessboard_port',
            default_value='/dev/ttyACM0',
            description='ChessBoard controller USB port'
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='/dev/ttyACM1',
            description='Robot controller USB port'
        ),
        DeclareLaunchArgument(
            'hardware_mode',
            default_value='mock',
            description='Hardware mode: mock or real'
        ),
        DeclareLaunchArgument(
            'stockfish_path',
            default_value='/usr/games/stockfish',
            description='Path to Stockfish executable'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable RViz2 visualization'
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Record rosbag2 data during testing'
        ),
    ]
    
    # Launch configuration
    test_mode = LaunchConfiguration('test_mode')
    chessboard_port = LaunchConfiguration('chessboard_port')
    robot_port = LaunchConfiguration('robot_port')
    hardware_mode = LaunchConfiguration('hardware_mode')
    stockfish_path = LaunchConfiguration('stockfish_path')
    log_level = LaunchConfiguration('log_level')
    enable_visualization = LaunchConfiguration('enable_visualization')
    record_bag = LaunchConfiguration('record_bag')
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch based on test mode"""
        
        test_mode_value = context.launch_configurations['test_mode']
        enable_viz = context.launch_configurations['enable_visualization'] == 'true'
        record_data = context.launch_configurations['record_bag'] == 'true'
        
        nodes = []
        
        # Common parameters for all nodes
        hardware_mode_value = context.launch_configurations['hardware_mode']
        use_mock = hardware_mode_value == 'mock'

        common_parameters = [
            config_file,
            {'hardware_mode': hardware_mode_value},
            {'use_mock_hardware': use_mock},
            {'chessboard_port': chessboard_port},
            {'robot_port': robot_port},
            {'stockfish_path': stockfish_path},
        ]
        
        # Core system nodes (using existing nodes)

        # Game Manager Node (RE-ENABLED to test if it causes service issues)
        nodes.append(
            Node(
                package='chessmate',
                executable='game_management_node',
                name='game_management_node',
                parameters=common_parameters,
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )

        # Arduino Communication Node (TEMPORARILY DISABLED for service debugging)
        # nodes.append(
        #     Node(
        #         package='chessmate',
        #         executable='arduino_communication_node',
        #         name='arduino_communication_node',
        #         parameters=common_parameters,
        #         arguments=['--ros-args', '--log-level', log_level],
        #         output='screen'
        #     )
        # )
        
        # Chess Engine Server Node
        if test_mode_value in ['full', 'game_simulation']:
            nodes.append(
                Node(
                    package='chessmate',
                    executable='chess_engine_server',
                    name='chess_engine_server',
                    parameters=common_parameters,
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            )
        
        # Test-specific nodes
        
        if test_mode_value == 'controllers':
            # Controller communication test
            nodes.append(
                TimerAction(
                    period=3.0,  # Wait 3 seconds for nodes to start
                    actions=[
                        Node(
                            package='chessmate',
                            executable='test_controller_communication',
                            name='test_controller_communication',
                            parameters=common_parameters,
                            arguments=['--ros-args', '--log-level', log_level],
                            output='screen'
                        )
                    ]
                )
            )
        
        elif test_mode_value == 'quick':
            # Quick integration test
            nodes.append(
                TimerAction(
                    period=5.0,  # Wait 5 seconds for all nodes to start
                    actions=[
                        Node(
                            package='chessmate',
                            executable='test_integration_quick',
                            name='test_integration_quick',
                            parameters=common_parameters,
                            arguments=['--ros-args', '--log-level', log_level],
                            output='screen'
                        )
                    ]
                )
            )
        
        elif test_mode_value == 'game_simulation':
            # End-to-end game simulation
            nodes.append(
                TimerAction(
                    period=10.0,  # Wait 10 seconds for all nodes to start
                    actions=[
                        Node(
                            package='chessmate',
                            executable='test_end_to_end_game',
                            name='test_end_to_end_game',
                            parameters=common_parameters + [
                                {'num_moves': 10},
                                {'game_timeout': 300}  # 5 minutes
                            ],
                            arguments=['--ros-args', '--log-level', log_level],
                            output='screen'
                        )
                    ]
                )
            )
        
        elif test_mode_value == 'full':
            # Comprehensive integration test suite
            nodes.append(
                TimerAction(
                    period=5.0,
                    actions=[
                        Node(
                            package='chessmate',
                            executable='test_integration_comprehensive',
                            name='test_integration_comprehensive',
                            parameters=common_parameters,
                            arguments=['--ros-args', '--log-level', log_level],
                            output='screen'
                        )
                    ]
                )
            )
        
        # Visualization nodes (optional) - disabled for headless Pi testing
        # Note: Visualization nodes are handled by host_visualization.launch.py
        # if enable_viz:
        #     # Visualization would go here for GUI environments
        
        # Data recording (optional - only if rosbag2 is available)
        if record_data:
            try:
                # Check if rosbag2 is available
                import subprocess
                result = subprocess.run(['ros2', 'bag', '--help'],
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    nodes.append(
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'bag', 'record',
                                '-o', f'/tmp/chessmate_test_{test_mode_value}',
                                '/chessmate/board_state',
                                '/chessmate/chess_moves',
                                '/chessmate/robot_status',
                                '/chessmate/game_state',
                                '/chessmate/human_input'
                            ],
                            name='rosbag_recorder',
                            output='screen'
                        )
                    )
                else:
                    print("Warning: rosbag2 not available, skipping data recording")
            except Exception as e:
                print(f"Warning: Could not start rosbag2 recording: {e}")
        
        return nodes
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


def generate_controller_test_launch():
    """Generate launch description for controller-only testing"""
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='debug'),
        
        Node(
            package='chessmate',
            executable='chessboard_interface_node',
            name='chessboard_interface_node',
            parameters=[
                {'hardware_mode': 'mock'},
                {'chessboard_port': '/dev/ttyACM0'}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        Node(
            package='chessmate',
            executable='robot_interface_node',
            name='robot_interface_node',
            parameters=[
                {'hardware_mode': 'mock'},
                {'robot_port': '/dev/ttyACM1'}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='chessmate',
                    executable='test_controller_communication',
                    name='test_controller_communication',
                    output='screen'
                )
            ]
        )
    ])
