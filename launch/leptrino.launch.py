import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    ft_sensor_params = os.path.join(
        get_package_share_directory('leptrino_force_torque'),
        'config', 'leptrino_force_torque_sensor.yaml')

    return LaunchDescription([
        LogInfo(msg='Launching leptrino force torque sensor node.'),
        Node(
            package='leptrino_force_torque',
            namespace="leptrino",
            executable='leptrino_force_torque_node',
            name='ft_sensor',
            output='screen',
            parameters=[ft_sensor_params]
        )
    ])
