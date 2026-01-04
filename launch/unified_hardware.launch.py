#!/usr/bin/env python3
"""
Unified Hardware Launch File for ChessMate

Cross-platform launch configuration for Linux host or Raspberry Pi deployment.
User must explicitly specify the platform and hardware mode.

Features:
- User-specified platform and hardware mode
- Conditional node launching based on platform
- Unified parameter loading
- Hardware testing integration
- Mock mode support for development

Usage:
  # For Raspberry Pi with real hardware
  ros2 launch chessmate_hardware unified_hardware.launch.py platform:=raspberry_pi hardware_mode:=real

  # For development machine with mock hardware
  ros2 launch chessmate_hardware unified_hardware.launch.py platform:=linux_host hardware_mode:=mock

  # With different log levels
  ros2 launch chessmate_hardware unified_hardware.launch.py platform:=raspberry_pi hardware_mode:=real log_level:=DEBUG
  ros2 launch chessmate_hardware unified_hardware.launch.py platform:=raspberry_pi hardware_mode:=real log_level:=ERROR
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory





def generate_launch_description():
    """Generate launch description based on platform and configuration"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('chessmate')
    config_file = os.path.join(pkg_dir, 'config', 'unified_hardware_config.yaml')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'platform',
            default_value='raspberry_pi',
            description='Target platform: linux_host, raspberry_pi, or mock (user must specify)'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to hardware configuration file'
        ),
        DeclareLaunchArgument(
            'hardware_mode',
            default_value='real',
            description='Hardware mode: real, mock, or simulation'
        ),
        DeclareLaunchArgument(
            'enable_testing',
            default_value='false',
            description='Enable hardware testing nodes'
        ),
        DeclareLaunchArgument(
            'auto_calibrate',
            default_value='false',
            description='Automatically calibrate on startup'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error'
        ),
    ]
    
    # Launch configuration
    platform = LaunchConfiguration('platform')
    config_file_arg = LaunchConfiguration('config_file')
    hardware_mode = LaunchConfiguration('hardware_mode')
    enable_testing = LaunchConfiguration('enable_testing')
    auto_calibrate = LaunchConfiguration('auto_calibrate')
    log_level = LaunchConfiguration('log_level')
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch based on context"""
        
        platform_value = context.launch_configurations['platform']
        hardware_mode_value = context.launch_configurations['hardware_mode']
        enable_testing_value = context.launch_configurations['enable_testing']
        
        nodes = []
        
        # Common parameters for all nodes
        use_mock_hardware = hardware_mode_value in ['mock', 'simulation']
        common_parameters = [
            config_file_arg,
            {'platform_override': platform_value},
            {'hardware_mode': hardware_mode_value},
            {'use_mock_hardware': use_mock_hardware},
            {'auto_calibrate': auto_calibrate},
        ]
        
        # Core hardware nodes (always launched)
        
        # Unified Arduino Bridge (replaces arduino_communication_node)
        nodes.append(
            Node(
                package='chessmate',
                executable='unified_arduino_bridge',
                name='unified_arduino_bridge',
                parameters=common_parameters,
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        # Game Management Node
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
        
        # Robot Animation Controller
        nodes.append(
            Node(
                package='chessmate',
                executable='robot_animation_controller',
                name='robot_animation_controller',
                parameters=common_parameters,
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        # Platform-specific nodes
        
        # Raspberry Pi specific nodes
        if platform_value == 'raspberry_pi':
            # LCD Display Node
            nodes.append(
                Node(
                    package='chessmate',
                    executable='lcd_display_node',
                    name='lcd_display_node',
                    parameters=common_parameters,
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            )
            
            # Rotary Encoder Node
            nodes.append(
                Node(
                    package='chessmate',
                    executable='rotary_encoder_node',
                    name='rotary_encoder_node',
                    parameters=common_parameters,
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            )
        
        # Testing nodes (optional)
        if enable_testing_value == 'true':
            nodes.append(
                Node(
                    package='chessmate',
                    executable='unified_hardware_test',
                    name='unified_hardware_test',
                    parameters=common_parameters + [
                        {'test_mode': 'comprehensive'},
                        {'auto_start_tests': True}
                    ],
                    arguments=['--ros-args', '--log-level', log_level, 'comprehensive'],
                    output='screen'
                )
            )
        
        return nodes
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


# Alternative simplified launch functions for specific scenarios

def generate_mock_launch_description():
    """Generate launch description for mock hardware development"""
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='debug'),

        Node(
            package='chessmate',
            executable='unified_arduino_bridge',
            name='unified_arduino_bridge',
            parameters=[
                {'hardware_mode': 'mock'},
                {'use_mock_hardware': True},
                {'platform_override': 'mock'},
                {'enable_debug_topics': True}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),

        Node(
            package='chessmate',
            executable='unified_hardware_test',
            name='unified_hardware_test',
            parameters=[
                {'test_mode': 'quick'},
                {'hardware_mode': 'mock'},
                {'use_mock_hardware': True}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level'), 'quick'],
            output='screen'
        )
    ])


def generate_raspberry_pi_launch_description():
    """Generate launch description specifically for Raspberry Pi"""
    pkg_dir = get_package_share_directory('chessmate')
    config_file = os.path.join(pkg_dir, 'config', 'unified_hardware_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('auto_calibrate', default_value='true'),
        
        # Core hardware bridge
        Node(
            package='chessmate',
            executable='unified_arduino_bridge',
            name='unified_arduino_bridge',
            parameters=[
                config_file,
                {'platform_override': 'raspberry_pi'},
                {'auto_calibrate': LaunchConfiguration('auto_calibrate')}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        # Raspberry Pi GPIO nodes
        Node(
            package='chessmate',
            executable='lcd_display_node',
            name='lcd_display_node',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        Node(
            package='chessmate',
            executable='rotary_encoder_node',
            name='rotary_encoder_node',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        # Game management
        Node(
            package='chessmate',
            executable='game_management_node',
            name='game_management_node',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        # Robot control
        Node(
            package='chessmate',
            executable='robot_animation_controller',
            name='robot_animation_controller',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        )
    ])


def generate_linux_host_launch_description():
    """Generate launch description specifically for Linux host development"""
    pkg_dir = get_package_share_directory('chessmate')
    config_file = os.path.join(pkg_dir, 'config', 'unified_hardware_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='debug'),
        DeclareLaunchArgument('hardware_mode', default_value='mock'),

        # Core hardware bridge with mock support
        Node(
            package='chessmate',
            executable='unified_arduino_bridge',
            name='unified_arduino_bridge',
            parameters=[
                config_file,
                {'platform_override': 'linux_host'},
                {'hardware_mode': LaunchConfiguration('hardware_mode')},
                {'enable_debug_topics': True}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        # Game management
        Node(
            package='chessmate',
            executable='game_management_node',
            name='game_management_node',
            parameters=[
                config_file,
                {'platform_override': 'linux_host'}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        # Robot control
        Node(
            package='chessmate',
            executable='robot_animation_controller',
            name='robot_animation_controller',
            parameters=[
                config_file,
                {'platform_override': 'linux_host'}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        )
    ])
