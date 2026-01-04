#!/usr/bin/env python3
"""
ChessMate Host Visualization Launch File

This launch file starts GUI and visualization components on the development host
that connect to the Pi's ROS2 nodes for enhanced development and debugging.

Designed for Phase 2 testing: Host-Pi distributed visualization and monitoring.

Usage:
  # Basic visualization (connect to Pi)
  ros2 launch chessmate_hardware host_visualization.launch.py

  # With custom Pi hostname
  ros2 launch chessmate_hardware host_visualization.launch.py pi_hostname:=chessmate-pi

  # With custom ROS domain
  ros2 launch chessmate_hardware host_visualization.launch.py ros_domain_id:=42

  # Full monitoring dashboard
  ros2 launch chessmate_hardware host_visualization.launch.py enable_dashboard:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for host visualization"""
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'pi_hostname',
            default_value='localhost',
            description='Hostname or IP of the Raspberry Pi running core nodes'
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='ROS Domain ID (must match Pi configuration)'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz2 3D visualization'
        ),
        DeclareLaunchArgument(
            'enable_dashboard',
            default_value='false',
            description='Enable monitoring dashboard'
        ),
        DeclareLaunchArgument(
            'enable_plotting',
            default_value='false',
            description='Enable real-time plotting tools'
        ),
    ]
    
    # Launch configuration
    pi_hostname = LaunchConfiguration('pi_hostname')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    log_level = LaunchConfiguration('log_level')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_dashboard = LaunchConfiguration('enable_dashboard')
    enable_plotting = LaunchConfiguration('enable_plotting')
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch based on configuration"""
        
        enable_rviz_value = context.launch_configurations['enable_rviz'] == 'true'
        enable_dashboard_value = context.launch_configurations['enable_dashboard'] == 'true'
        enable_plotting_value = context.launch_configurations['enable_plotting'] == 'true'
        
        nodes = []
        
        # Common parameters for host nodes
        common_parameters = [
            {'pi_hostname': pi_hostname},
            {'ros_domain_id': ros_domain_id},
            {'host_mode': True},
        ]
        
        # Chess Board Visualization Node (3D board representation)
        if enable_rviz_value:
            # Get RViz config file
            pkg_dir = get_package_share_directory('chessmate')
            rviz_config = os.path.join(pkg_dir, 'rviz', 'chessmate_visualization.rviz')
            
            nodes.append(
                Node(
                    package='chessmate',
                    executable='chess_visualization_node',
                    name='chess_visualization_node',
                    parameters=common_parameters,
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            )
            
            # RViz2 for 3D visualization
            nodes.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                    parameters=[
                        {'use_sim_time': False},
                    ],
                    output='screen'
                )
            )
        
        # System Monitoring Dashboard
        if enable_dashboard_value:
            nodes.append(
                Node(
                    package='chessmate',
                    executable='system_monitor_node',
                    name='system_monitor_node',
                    parameters=common_parameters + [
                        {'monitor_topics': [
                            '/board_state',
                            '/robot_status', 
                            '/chess_moves',
                            '/game_state'
                        ]},
                        {'update_rate': 10.0},  # 10 Hz monitoring
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            )
            
            # rqt Dashboard for system monitoring
            nodes.append(
                Node(
                    package='rqt_gui',
                    executable='rqt_gui',
                    name='rqt_dashboard',
                    arguments=['--perspective-file', 
                              os.path.join(pkg_dir, 'rqt', 'chessmate_dashboard.perspective')],
                    output='screen'
                )
            )
        
        # Real-time Plotting Tools
        if enable_plotting_value:
            # Performance metrics plotting
            nodes.append(
                Node(
                    package='chessmate',
                    executable='performance_plotter_node',
                    name='performance_plotter_node',
                    parameters=common_parameters + [
                        {'plot_topics': [
                            '/robot_status',
                            '/timing_metrics',
                            '/system_health'
                        ]},
                        {'plot_window_size': 100},  # Last 100 data points
                    ],
                    arguments=['--ros-args', '--log-level', log_level],
                    output='screen'
                )
            )
            
            # rqt_plot for real-time data visualization
            nodes.append(
                Node(
                    package='rqt_plot',
                    executable='rqt_plot',
                    name='rqt_plot',
                    arguments=[
                        '/robot_status/move_duration',
                        '/timing_metrics/service_latency',
                        '/system_health/cpu_usage'
                    ],
                    output='screen'
                )
            )
        
        # Game State Monitor (always enabled for host)
        nodes.append(
            Node(
                package='chessmate',
                executable='game_state_monitor_node',
                name='game_state_monitor_node',
                parameters=common_parameters + [
                    {'display_mode': 'gui'},  # GUI mode for host
                    {'auto_refresh': True},
                    {'refresh_rate': 2.0},  # 2 Hz refresh
                ],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        # Topic Bridge (if needed for cross-network communication)
        nodes.append(
            Node(
                package='chessmate',
                executable='topic_bridge_node',
                name='topic_bridge_node',
                parameters=common_parameters + [
                    {'bridge_topics': [
                        '/board_state',
                        '/robot_status',
                        '/chess_moves',
                        '/game_state',
                        '/human_input',
                        '/engine_evaluation'
                    ]},
                    {'bridge_mode': 'subscriber'},  # Host subscribes to Pi topics
                ],
                arguments=['--ros-args', '--log-level', log_level],
                output='screen'
            )
        )
        
        return nodes
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


def generate_minimal_host_launch():
    """Generate minimal host launch for basic monitoring"""
    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        DeclareLaunchArgument('log_level', default_value='info'),
        
        # Basic game state monitor
        Node(
            package='chessmate',
            executable='game_state_monitor_node',
            name='game_state_monitor_node',
            parameters=[
                {'display_mode': 'terminal'},  # Terminal output
                {'ros_domain_id': LaunchConfiguration('ros_domain_id')},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),
        
        # Topic echo for debugging
        Node(
            package='chessmate',
            executable='topic_echo_node',
            name='topic_echo_node',
            parameters=[
                {'echo_topics': ['/board_state', '/robot_status', '/chess_moves']},
                {'ros_domain_id': LaunchConfiguration('ros_domain_id')},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        )
    ])


def generate_rviz_only_launch():
    """Generate RViz-only launch for 3D visualization"""
    pkg_dir = get_package_share_directory('chessmate')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'chessmate_visualization.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        
        # Chess visualization node
        Node(
            package='chessmate',
            executable='chess_visualization_node',
            name='chess_visualization_node',
            parameters=[
                {'ros_domain_id': LaunchConfiguration('ros_domain_id')},
            ],
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            parameters=[
                {'use_sim_time': False},
            ],
            output='screen'
        )
    ])
