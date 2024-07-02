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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    honeybee_nav_dir = get_package_share_directory('honeybee_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    slam_cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': use_sim_time}],
        arguments = [
            '-configuration_directory', os.path.join(honeybee_nav_dir, 'config'),
            '-configuration_basename', 'cartographer_slam.lua'],
        remappings = [('points', '/sensors/lidar_0/points')],
        output = 'screen',
        condition=IfCondition(slam)
    )

    localization_cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': use_sim_time}],
        arguments = [
            '-configuration_directory', os.path.join(honeybee_nav_dir, 'config'),
            '-configuration_basename', 'cartographer_localization.lua',
            '-load_state_filename', 'PATH/TO/SLAM/TODO'], #TODO
        remappings = [('points', '/sensors/lidar_0/points')],
        output = 'screen',
        condition=IfCondition(PythonExpression(['not ', slam]))
    )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(slam_cartographer_node)
    ld.add_action(localization_cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    return ld
