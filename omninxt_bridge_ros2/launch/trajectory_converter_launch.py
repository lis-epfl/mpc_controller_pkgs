#!/usr/bin/env python3
"""
Launch file for trajectory converter node only
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare('omninxt_bridge_ros2'),
        'config',
        'bridge_params.yaml'
    ])
    
    # Launch arguments
    traj_input_topic_arg = DeclareLaunchArgument(
        'traj_input_topic',
        default_value='/agent_0/traj_full',
        description='Input topic for trajectory (multi_agent_planner_msgs)'
    )
    
    traj_output_topic_arg = DeclareLaunchArgument(
        'traj_output_topic',
        default_value='/planner/trajectory',
        description='Output topic for trajectory (mpc_controller_ros2_msgs)'
    )
    
    # Trajectory converter node
    trajectory_converter_node = Node(
        package='omninxt_bridge_ros2',
        executable='trajectory_converter_node',
        name='trajectory_converter',
        output='screen',
        parameters=[
            config_file,
            {
                'input_topic': LaunchConfiguration('traj_input_topic'),
                'output_topic': LaunchConfiguration('traj_output_topic'),
            }
        ]
    )
    
    return LaunchDescription([
        traj_input_topic_arg,
        traj_output_topic_arg,
        trajectory_converter_node,
    ])
