#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('px4_sim_bridge_ros2'),
            'config',
            'bridge_params.yaml'
        ]),
        description='Path to bridge configuration file'
    )
    
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='true',
        description='Enable automatic arming'
    )
    
    auto_offboard_arg = DeclareLaunchArgument(
        'auto_offboard',
        default_value='true',
        description='Enable automatic offboard mode switching'
    )
    
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='1.0',
        description='Takeoff altitude in meters'
    )
    
    # Create the bridge node
    bridge_node = Node(
        package='px4_sim_bridge_ros2',
        executable='px4_sim_bridge_node',
        name='px4_sim_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'auto_arm': LaunchConfiguration('auto_arm'),
                'auto_offboard': LaunchConfiguration('auto_offboard'),
                'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            }
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        auto_arm_arg,
        auto_offboard_arg,
        takeoff_altitude_arg,
        bridge_node
    ])
