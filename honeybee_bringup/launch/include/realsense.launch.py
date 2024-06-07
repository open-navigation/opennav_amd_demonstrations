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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'config', 'realsense.yaml']))

    realsense2_camera_node = Node(
        package='realsense2_camera',
        name='intel_realsense',
        executable='realsense2_camera_node',
        parameters=[parameters],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(realsense2_camera_node)
    return ld
