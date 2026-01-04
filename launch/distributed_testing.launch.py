#!/usr/bin/env python3
"""
ChessMate Distributed Testing Launch File

This launch file coordinates distributed testing across Pi and Host systems.
It provides a unified interface for launching both Pi and Host components
with proper synchronization and configuration.

Usage:
  # On Raspberry Pi (Phase 1 - Core functionality)
  ros2 launch chessmate_hardware distributed_testing.launch.py \
      role:=pi test_mode:=comprehensive

  # On Development Host (Phase 2 - Add visualization)  
  ros2 launch chessmate_hardware distributed_testing.launch.py \
      role:=host pi_hostname:=chessmate-pi.local

  # Coordinated launch (if using shared launch system)
  ros2 launch chessmate_hardware distributed_testing.launch.py \
      role:=coordinator pi_hostname:=chessmate-pi.local test_mode:=game_simulation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for distributed testing"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('chessmate')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'role',
            default_value='pi',
            description='System role: pi, host, or coordinator'
        ),
        DeclareLaunchArgument(
            'test_mode',
            default_value='comprehensive',
            description='Test mode: controllers, quick, comprehensive, game_simulation'
        ),
        DeclareLaunchArgument(
            'pi_hostname',
            default_value='localhost',
            description='Hostname or IP of the Raspberry Pi'
        ),
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='ROS Domain ID for distributed communication'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable visualization components (host only)'
        ),
        DeclareLaunchArgument(
            'enable_monitoring',
            default_value='true',
            description='Enable system monitoring'
        ),
        DeclareLaunchArgument(
            'sync_launch',
            default_value='false',
            description='Enable launch synchronization between systems'
        ),
    ]
    
    # Launch configuration
    role = LaunchConfiguration('role')
    test_mode = LaunchConfiguration('test_mode')
    pi_hostname = LaunchConfiguration('pi_hostname')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    log_level = LaunchConfiguration('log_level')
    enable_visualization = LaunchConfiguration('enable_visualization')
    enable_monitoring = LaunchConfiguration('enable_monitoring')
    sync_launch = LaunchConfiguration('sync_launch')
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch based on role"""
        
        role_value = context.launch_configurations['role']
        test_mode_value = context.launch_configurations['test_mode']
        enable_viz = context.launch_configurations['enable_visualization'] == 'true'
        enable_mon = context.launch_configurations['enable_monitoring'] == 'true'
        
        launch_descriptions = []
        
        if role_value == 'pi':
            # Raspberry Pi role: Launch headless core nodes
            pi_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_dir, 'launch', 'pi_headless_testing.launch.py')
                ]),
                launch_arguments={
                    'test_mode': test_mode,
                    'ros_domain_id': ros_domain_id,
                    'log_level': log_level,
                    'record_bag': 'true',  # Always record on Pi
                }.items()
            )
            launch_descriptions.append(pi_launch)
            
        elif role_value == 'host':
            # Development Host role: Launch visualization and monitoring
            if enable_viz:
                host_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_dir, 'launch', 'host_visualization.launch.py')
                    ]),
                    launch_arguments={
                        'pi_hostname': pi_hostname,
                        'ros_domain_id': ros_domain_id,
                        'log_level': log_level,
                        'enable_rviz': 'true',
                        'enable_dashboard': str(enable_mon).lower(),
                        'enable_plotting': 'true',
                    }.items()
                )
                launch_descriptions.append(host_launch)
            
        elif role_value == 'coordinator':
            # Coordinator role: Launch both Pi and Host components
            # Note: This requires proper network setup and SSH access
            
            # Pi components (would typically be launched via SSH)
            pi_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_dir, 'launch', 'pi_headless_testing.launch.py')
                ]),
                launch_arguments={
                    'test_mode': test_mode,
                    'ros_domain_id': ros_domain_id,
                    'log_level': log_level,
                }.items()
            )
            launch_descriptions.append(pi_launch)
            
            # Host components
            if enable_viz:
                host_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(pkg_dir, 'launch', 'host_visualization.launch.py')
                    ]),
                    launch_arguments={
                        'pi_hostname': pi_hostname,
                        'ros_domain_id': ros_domain_id,
                        'log_level': log_level,
                        'enable_rviz': 'true',
                        'enable_dashboard': str(enable_mon).lower(),
                    }.items()
                )
                launch_descriptions.append(host_launch)
        
        return launch_descriptions
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])


def generate_phase1_pi_launch():
    """Generate Phase 1 launch: Pi standalone testing"""
    pkg_dir = get_package_share_directory('chessmate')
    
    return LaunchDescription([
        DeclareLaunchArgument('test_mode', default_value='comprehensive'),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'pi_headless_testing.launch.py')
            ]),
            launch_arguments={
                'test_mode': LaunchConfiguration('test_mode'),
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'log_level': 'info',
                'record_bag': 'true',
            }.items()
        )
    ])


def generate_phase2_distributed_launch():
    """Generate Phase 2 launch: Pi + Host distributed testing"""
    pkg_dir = get_package_share_directory('chessmate')
    
    return LaunchDescription([
        DeclareLaunchArgument('pi_hostname', default_value='chessmate-pi.local'),
        DeclareLaunchArgument('test_mode', default_value='game_simulation'),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        
        # This would typically be run on the host, connecting to Pi
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_dir, 'launch', 'host_visualization.launch.py')
            ]),
            launch_arguments={
                'pi_hostname': LaunchConfiguration('pi_hostname'),
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'enable_rviz': 'true',
                'enable_dashboard': 'true',
                'enable_plotting': 'true',
            }.items()
        )
    ])
