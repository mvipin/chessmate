#!/usr/bin/env python3
"""
ChessMate Pi Headless Testing Launch File

This launch file starts only the core ROS2 nodes required for Raspberry Pi
headless testing, excluding GUI components like RViz2 that require a display.

Designed for Phase 1 testing: Pi standalone validation of core functionality.

Usage:
  # Controller communication test
  ros2 launch chessmate_hardware pi_headless_testing.launch.py test_mode:=controllers

  # Quick integration test  
  ros2 launch chessmate_hardware pi_headless_testing.launch.py test_mode:=quick

  # Comprehensive integration test
  ros2 launch chessmate_hardware pi_headless_testing.launch.py test_mode:=comprehensive

  # End-to-end game simulation
  ros2 launch chessmate_hardware pi_headless_testing.launch.py test_mode:=game_simulation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Pi headless testing"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('chessmate')
    config_file = os.path.join(pkg_dir, 'config', 'unified_hardware_config.yaml')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'test_mode',
            default_value='comprehensive',
            description='Test mode: controllers, quick, comprehensive, game_simulation'
        ),
        DeclareLaunchArgument(
            'chessboard_port',
            default_value='/dev/chessboard',
            description='ChessBoard controller USB port (udev symlink)'
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='/dev/robot',
            description='Robot controller USB port (udev symlink)'
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
            'record_bag',
            default_value='false',
            description='Record rosbag2 data during testing'
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='ROS Domain ID for distributed setup'
        ),
    ]
    
    # Launch configuration
    test_mode = LaunchConfiguration('test_mode')
    chessboard_port = LaunchConfiguration('chessboard_port')
    robot_port = LaunchConfiguration('robot_port')
    stockfish_path = LaunchConfiguration('stockfish_path')
    log_level = LaunchConfiguration('log_level')
    record_bag = LaunchConfiguration('record_bag')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch based on test mode"""
        
        test_mode_value = context.launch_configurations['test_mode']
        record_data = context.launch_configurations['record_bag'] == 'true'
        
        nodes = []
        
        # Common parameters for all nodes (Pi-specific)
        common_parameters = [
            config_file,
            {'hardware_mode': 'mock'},
            {'use_mock_hardware': True},
            {'platform_override': 'raspberry_pi'},
            {'chessboard_port': chessboard_port},
            {'robot_port': robot_port},
            {'stockfish_path': stockfish_path},
            {'headless_mode': True},  # Pi-specific: no GUI components
            {'ros_domain_id': ros_domain_id},
        ]
        
        # Core Pi nodes (always launched for headless operation)
        
        # ChessBoard Interface Node (USB Serial wrapper)
        nodes.append(
            Node(
                package='chessmate',
                executable='chessboard_interface_node',
                name='chessboard_interface_node',
                parameters=common_parameters,
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        # Robot Interface Node (USB Serial wrapper)
        nodes.append(
            Node(
                package='chessmate',
                executable='robot_interface_node',
                name='robot_interface_node',
                parameters=common_parameters,
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        # Game Manager Node (core game logic)
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
        
        # Human Interface Simulation Node (headless simulation)
        nodes.append(
            Node(
                package='chessmate',
                executable='human_interface_simulation_node',
                name='human_interface_simulation_node',
                parameters=common_parameters + [
                    {'headless_simulation': True}  # No LCD/encoder GUI simulation
                ],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        # Chess Engine Server Node (for game simulation)
        if test_mode_value in ['comprehensive', 'game_simulation']:
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
        
        # Test-specific nodes (Pi headless versions)
        
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
        
        elif test_mode_value == 'comprehensive':
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
        
        # Data recording (optional, Pi-optimized)
        if record_data:
            nodes.append(
                Node(
                    package='rosbag2_py',
                    executable='record',
                    name='rosbag_recorder',
                    arguments=[
                        'record',
                        '-o', f'/tmp/chessmate_pi_test_{test_mode_value}',
                        '/board_state',
                        '/chess_moves',
                        '/robot_status',
                        '/game_state',
                        '/human_input',
                        '/engine_evaluation'
                    ],
                    output='screen'
                )
            )
        
        return nodes
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


def generate_pi_controller_test_launch():
    """Generate launch description for Pi controller-only testing"""
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        
        Node(
            package='chessmate',
            executable='chessboard_interface_node',
            name='chessboard_interface_node',
            parameters=[
                {'hardware_mode': 'mock'},
                {'platform_override': 'raspberry_pi'},
                {'chessboard_port': '/dev/ttyACM0'},
                {'headless_mode': True},
                {'ros_domain_id': LaunchConfiguration('ros_domain_id')},
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
                {'platform_override': 'raspberry_pi'},
                {'robot_port': '/dev/ttyACM1'},
                {'headless_mode': True},
                {'ros_domain_id': LaunchConfiguration('ros_domain_id')},
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
                    parameters=[
                        {'ros_domain_id': LaunchConfiguration('ros_domain_id')},
                    ],
                    output='screen'
                )
            ]
        )
    ])
