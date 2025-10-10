#!/usr/bin/env python3
"""
Launch file for command converter, trajectory converter, and MPC controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Configuration files
    bridge_config_file = PathJoinSubstitution([
        FindPackageShare('omninxt_bridge_ros2'),
        'config',
        'bridge_params.yaml'
    ])
    
    omninxt_config_file = PathJoinSubstitution([
        FindPackageShare('omninxt_bridge_ros2'),
        'config',
        'omninxt_params.yaml'
    ])
    
    # Launch arguments
    traj_input_topic_arg = DeclareLaunchArgument(
        'traj_input_topic',
        default_value='/planner/trajectory',
        description='Input topic for trajectory (multi_agent_planner_msgs)'
    )
    
    traj_output_topic_arg = DeclareLaunchArgument(
        'traj_output_topic',
        default_value='/mpc/trajectory',
        description='Output topic for trajectory (mpc_controller_ros2_msgs)'
    )
    
    cmd_output_topic_arg = DeclareLaunchArgument(
        'cmd_output_topic',
        default_value='/controller/cmd',
        description='Output topic for controller commands (swarmnxt_msgs)'
    )
    
    # MPC Controller node with remappings and config
    mpc_controller_node = Node(
        package='mpc_controller_ros2',
        executable='mpc_controller_node',
        name='mpc_controller',
        output='screen',
        parameters=[omninxt_config_file],
        remappings=[
            ('/fmu/in/vehicle_rates_setpoint', '/mpc/out/vehicle_rates_setpoint'),
            ('/fmu/in/actuator_motors', '/mpc/out/actuator_motors'),
        ]
    )
    
    # Trajectory converter node
    trajectory_converter_node = Node(
        package='omninxt_bridge_ros2',
        executable='trajectory_converter_node',
        name='trajectory_converter',
        output='screen',
        parameters=[
            bridge_config_file,
            {
                'input_topic': LaunchConfiguration('traj_input_topic'),
                'output_topic': LaunchConfiguration('traj_output_topic'),
            }
        ]
    )
    
    # Command converter node
    command_converter_node = Node(
        package='omninxt_bridge_ros2',
        executable='command_converter_node',
        name='command_converter',
        output='screen',
        parameters=[
            bridge_config_file,
            {
                'output_topic': LaunchConfiguration('cmd_output_topic'),
            }
        ]
    )
    
    return LaunchDescription([
        traj_input_topic_arg,
        traj_output_topic_arg,
        cmd_output_topic_arg,
        mpc_controller_node,
        trajectory_converter_node,
        command_converter_node,
    ])
