import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ft_sensor_params = os.path.join(
        get_package_share_directory('leptrino_force_torque'),
        'config', 'leptrino_force_torque_sensor.yaml')

    leptrino_ft_sensor_composable_node = ComposableNode(
        package='leptrino_force_torque',
        namespace='leptrino',
        plugin='leptrino::ForceTorqueSensorNode',
        name='ft_sensor',
        parameters=[ft_sensor_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    return LaunchDescription([
        LogInfo(msg='Launching leptrino force torque sensor node.'),
        ComposableNodeContainer(
            name='leptrino_force_torque_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[leptrino_ft_sensor_composable_node]
        )
    ])
