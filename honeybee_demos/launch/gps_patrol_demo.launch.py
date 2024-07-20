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

import os 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    honeybee_nav2 = get_package_share_directory('honeybee_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation or hardware'
    )

    param_file = os.path.join(honeybee_nav2, 'config', 'nav2_gps_params.yaml')
    bt_xml = os.path.join(honeybee_nav2, 'behavior_trees', 'honeybee_gps_bt.xml')

    # Nav2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                honeybee_nav2, 'launch', 'nav2.launch.py'])]),
        launch_arguments={'use_sim_time': use_sim_time,
                          'localization_type': 'GPS',
                          'params_file': param_file,
                          'nav2pose_bt_xml': bt_xml}.items(),
    )

    # Autonomy demo
    demo_script_cmd = Node(
        package='honeybee_demos',
        executable='gps_patrol_demo',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Data capture
    rosbag_record_cmd = Node(
        package='honeybee_watchdogs',
        executable='record_rosbag_data',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    capture_metrics_cmd = Node(
        package='honeybee_watchdogs',
        executable='capture_system_metrics',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Clean poweroff for backpack computer
    power_off_cmd = Node(
        package='honeybee_watchdogs',
        executable='power_off_backpack',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(demo_script_cmd)
    ld.add_action(rosbag_record_cmd)
    ld.add_action(power_off_cmd)
    ld.add_action(capture_metrics_cmd)
    ld.add_action(nav2_cmd)
    return ld
