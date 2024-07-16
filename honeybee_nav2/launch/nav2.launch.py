# Copyright (C) 2024 Open Navigation LLC
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(bringup_dir, 'launch')
    honeybee_nav_dir = get_package_share_directory('honeybee_nav2')
    launch_dir = os.path.join(honeybee_nav_dir, 'launch')
    honeybee_launch_dir = os.path.join(honeybee_nav_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    local_nav = LaunchConfiguration('local_nav')
    localization_type = LaunchConfiguration('localization_type')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declare the launch arguments
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(honeybee_nav_dir, 'config', 'nav2_indoor_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_local_nav_cmd = DeclareLaunchArgument(
        'local_nav',
        default_value='False',
        description='Whether to disable all localization and use odom only',
    )

    declare_localization_type_cmd = DeclareLaunchArgument(
        'localization_type',
        default_value='2D',
        description='Whether to use indoor (2D), outdoor (3D), or GPS (GPS) localization',
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(honeybee_launch_dir, 'localization.launch.py')),
            launch_arguments={'map': map_yaml_file,
                              'slam': slam,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'local_nav': local_nav,
                              'localization_type': localization_type,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'use_composition': 'True',
                              'container_name': 'nav2_container'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_local_nav_cmd)
    ld.add_action(declare_localization_type_cmd)
    ld.add_action(bringup_cmd_group)
    return ld
