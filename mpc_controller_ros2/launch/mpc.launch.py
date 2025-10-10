import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mpc_controller_ros2'
    pkg_share = get_package_share_directory(pkg_name)
    params_file = os.path.join(pkg_share, 'config', 'mpc_params.yaml')

    mpc_node = Node(
        package=pkg_name,
        executable='mpc_controller_node',   # Executable name stays the same
        name='mpc_controller',               # Updated to match Node constructor name
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )

    return LaunchDescription([
        mpc_node
    ])
