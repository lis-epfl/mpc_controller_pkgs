import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'mpc_controller_ros2'
    pkg_share = get_package_share_directory(pkg_name)
    params_file = os.path.join(pkg_share, 'config', 'mpc_ominixt_params.yaml')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='nxt0',
        description='Namespace for the MPC controller'
    )

    mpc_node = Node(
        package=pkg_name,
        executable='mpc_controller_node',
        name='mpc_controller',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        remappings=[
            ('/fmu/in/vehicle_rates_setpoint', '/mpc/out/vehicle_rates_setpoint'),
            ('/fmu/in/vehicle_torque_setpoint', '/mpc/out/vehicle_torque_setpoint'),
            ('/fmu/in/vehicle_thrust_setpoint', '/mpc/out/vehicle_thrust_setpoint'),
            ('/fmu/in/actuator_motors', '/mpc/out/actuator_motors'),
        ]
    )

    return LaunchDescription([
        namespace_arg,  # ADD THIS - the argument declaration must be included!
        mpc_node
    ])
