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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Get the description package location to obtain files
    description_dir = get_package_share_directory('honeybee_bringup')

    # Get the appropriate Rviz configuration file
    rviz_config_file = LaunchConfiguration('rviz_config')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(description_dir, 'rviz', 'config.rviz'),
        description='Full path to the RVIZ config file to use')

    # Launch rviz to visualize the robot
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}])

    # Create the launch description and populate it
    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_rviz_cmd)
    return ld
