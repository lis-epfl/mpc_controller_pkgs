#!/usr/bin/env python3
"""
Simple launch file for PX4 simulation with MPC controller
Uses default configurations - ideal for quick testing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Bridge configuration
    bridge_config = PathJoinSubstitution([
        FindPackageShare('px4_sim_bridge_ros2'),
        'config',
        'bridge_params.yaml'
    ])
    
    # MPC configuration
    mpc_config = PathJoinSubstitution([
        FindPackageShare('mpc_controller_ros2'),
        'config',
        'mpc_params.yaml'
    ])
    
    # PX4 Simulation Bridge
    bridge_node = Node(
        package='px4_sim_bridge_ros2',
        executable='px4_sim_bridge_node',
        name='px4_sim_bridge',
        output='screen',
        parameters=[bridge_config]
    )
    
    # MPC Controller
    mpc_node = Node(
        package='mpc_controller_ros2',
        executable='mpc_controller_node',
        name='mpc_controller',
        # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
        output='screen',
        parameters=[mpc_config]
    )
    
    return LaunchDescription([
        bridge_node,
        mpc_node,
    ])
