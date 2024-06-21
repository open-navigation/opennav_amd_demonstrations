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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml


def generate_launch_description():
    # Input parameters declaration
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')
    honeybee_nav_dir = get_package_share_directory('honeybee_nav2')
    params_file = os.path.join(honeybee_nav_dir, 'config', 'slam_toolbox.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Nodes launching commands
    start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[configured_params])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_saver']}])


    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': params_file}.items())

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_slam_toolbox_cmd_with_params)

    return ld
