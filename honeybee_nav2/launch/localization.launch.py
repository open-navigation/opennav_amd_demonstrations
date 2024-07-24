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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(bringup_dir, 'launch')
    honeybee_nav_dir = get_package_share_directory('honeybee_nav2')
    honeybee_launch_dir = os.path.join(honeybee_nav_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    localization_type = LaunchConfiguration('localization_type')
    container_name = LaunchConfiguration('container_name')

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

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(honeybee_nav_dir, 'config', 'nav2_indoor_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_localization_type_cmd = DeclareLaunchArgument(
        'localization_type',
        default_value='2D',
        description='Whether to use indoor (2D), outdoor (3D), or GPS (GPS) localization',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    localization_cmd_group = GroupAction([
        # 2D SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(honeybee_launch_dir, 'include', 'slam_toolbox.launch.py')),
            condition=IfCondition(PythonExpression([slam, " and '", localization_type, "'=='2D'"])),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # 2D Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(honeybee_launch_dir, 'include', 'amcl.launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam, " and '", localization_type, "'=='2D'"])),
            launch_arguments={'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'container_name': container_name}.items()),

        # if local_nav, map->odom is unity static transform
        Node(
            package="tf2_ros", 
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0", "0", "0", "map", "odom"],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression(["'", localization_type, "'=='local'"]))
        ),

        # GPS Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(honeybee_launch_dir, 'include', 'gps_localization.launch.py')),
            condition=IfCondition(PythonExpression(["'", localization_type, "'=='GPS'"])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': os.path.join(honeybee_nav_dir, 'config', 'gps_localization.yaml')}.items()),

        # 3D SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(honeybee_launch_dir, 'include', '3d_localization.launch.py')),
            condition=IfCondition(PythonExpression([slam, " and '", localization_type, "'=='3D'"])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': os.path.join(honeybee_nav_dir, 'config', '3d_localization.yaml'),
                              'slam': 'True'}.items()),

        # 3D Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(honeybee_launch_dir, 'include', '3d_localization.launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam, " and '", localization_type, "'=='3D'"])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': os.path.join(honeybee_nav_dir, 'config', '3d_localization.yaml'),
                              'slam': 'False'}.items()),

    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_localization_type_cmd)
    ld.add_action(declare_container_name_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(localization_cmd_group)

    return ld
