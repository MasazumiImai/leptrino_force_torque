# Copyright (c) 2026 Tohoku Univ. Space Robotics Lab.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode


def generate_ft_sensor_nodes(limb_names):
    """Return ComposableNode list of Leptrino FT sensors."""

    lbr_ft_sensor_params_path = os.path.join(
        get_package_share_directory('leptrino_force_torque'),
        'config', 'lbr_ft_sensor_params.yaml')

    ft_sensor_config = {}
    try:
        with open(lbr_ft_sensor_params_path, 'r') as f:
            ft_sensor_config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading FT sensor params: {e}")
        return []

    if not ft_sensor_config.get('use_ft_sensor', False):
        print("[leptrino_ft_sensor] use_ft_sensor is false."
              "Skipping FT nodes.")
        return []

    ft_sensor_nodes = []

    for limb in limb_names:
        ft_sensor_node_name = f"/{limb}/ft_sensor"

        specific_ft_sensor_params = {}
        if ft_sensor_node_name in ft_sensor_config:
            specific_ft_sensor_params = ft_sensor_config[
                ft_sensor_node_name].get('ros__parameters', {})
        else:
            print("Warning: No params found for"
                  f"{ft_sensor_node_name} in yaml")

        node = ComposableNode(
            package='leptrino_force_torque',
            plugin='leptrino::ForceTorqueSensorNode',
            name='ft_sensor',
            namespace=limb,
            parameters=[specific_ft_sensor_params],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

        ft_sensor_nodes.append(node)

    return ft_sensor_nodes
