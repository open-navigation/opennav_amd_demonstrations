# Copyright (c) 2024 Open Navigation LLC
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
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import launch_ros.actions
from launch_ros.descriptions import ParameterFile

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    honeybee_nav_dir = get_package_share_directory('honeybee_nav2')
    honeybee_launch_dir = os.path.join(honeybee_nav_dir, 'launch')

    params_file = LaunchConfiguration('params_file')
    params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(honeybee_nav_dir, 'config', 'gps_localization.yaml')
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    return LaunchDescription([
        params_file_cmd,
        declare_use_sim_time_cmd,
        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_node_map',
            output='screen',
            parameters=[configured_params],
            remappings=[('odometry/filtered', '/ekf_node_map/odometry/global')]
        ),
        Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
            output='screen',
            parameters=[configured_params],
            remappings=[('imu', '/sensors/imu_1/data'),
                        ('gps/fix', '/sensors/gps_0/fix'),
                        ('odometry/gps', '/sensors/gps_0/odometry'),
                        ('odometry/filtered', '/ekf_node_map/odometry/global')]
        )
    ])
