from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF file path for complete ChessMate system
    urdf_file = os.path.join(
        get_package_share_directory('chessmate'),
        'urdf', 'chessmate_complete.urdf.xacro'
    )

    # Process Xacro file to generate URDF
    robot_desc = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use Joint State Publisher GUI'
        ),
        
        DeclareLaunchArgument(
            'use_interactive_markers',
            default_value='false',  # Disabled by default for complete system
            description='Enable interactive markers for pose adjustment'
        ),
        
        DeclareLaunchArgument(
            'debug_tf',
            default_value='false',
            description='Enable TF debugging output'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui'))
        ),
        
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
        
        # Custom interactive marker server for real-time pose adjustment
        Node(
            package='chessmate',
            executable='link_pose_adjuster.py',
            name='link_pose_adjuster',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_interactive_markers'))
        ),
        
        # TF2 tools for debugging transforms
        Node(
            package='tf2_ros',
            executable='tf2_echo',
            name='tf2_echo_debug',
            arguments=['world', 'end_effector'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('debug_tf'))
        ),
        
        # Static transform for table/workspace reference (optional)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='table_frame',
            arguments=['0', '0', '-0.05', '0', '0', '0', 'world', 'table'],
            output='screen'
        ),
    ])
