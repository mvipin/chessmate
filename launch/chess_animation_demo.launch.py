#!/usr/bin/env python3
"""
Launch file for ChessMate Animation Demo

This launch file starts the complete ChessMate system with animated
trajectory demonstration showing the robot reaching various chess squares.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('chessmate'),
        'urdf', 'chessmate_complete.urdf.xacro'
    )

    # Process Xacro file to generate URDF
    robot_desc = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_gui',
            default_value='false',
            description='Show Joint State Publisher GUI (disable for animation demo)'
        ),
        
        DeclareLaunchArgument(
            'animation_speed',
            default_value='1.0',
            description='Animation speed multiplier (1.0 = normal speed)'
        ),
        
        DeclareLaunchArgument(
            'loop_demo',
            default_value='true',
            description='Loop the demo sequence continuously'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # Joint State Publisher GUI (optional, disabled by default for demo)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('show_gui')),
            output='screen'
        ),

        # RViz2 with ChessMate configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('chessmate'),
                'rviz', 'chessmate_complete.rviz'
            )],
            output='screen'
        ),

        # ChessMate Animation Demo Node
        Node(
            package='chessmate',
            executable='chess_animation_demo.py',
            name='chess_animation_demo',
            output='screen',
            parameters=[
                {'animation_speed': LaunchConfiguration('animation_speed')},
                {'loop_demo': LaunchConfiguration('loop_demo')}
            ]
        ),

        # Static transform for table reference
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='table_frame',
            arguments=['0', '0', '-0.05', '0', '0', '0', 'world', 'table'],
            output='screen'
        ),

        # Note: Kinematics server disabled - animation demo has built-in kinematics
    ])
