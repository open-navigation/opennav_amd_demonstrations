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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_microstrain_inertial_driver = FindPackageShare('microstrain_inertial_driver')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=os.path.join(
          get_package_share_directory('honeybee_bringup'), 'config', 'microstrain.yaml'))

    launch_microstrain_imu = PathJoinSubstitution([
        pkg_microstrain_inertial_driver, 'launch', 'microstrain_launch.py'])

    launch_microstrain_imu = GroupAction([
        SetRemap('imu/data', 'sensors/imu_1/data'),
        SetRemap('/moving_ang', 'sensors/imu_1/moving_ang'),

        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([launch_microstrain_imu]),
          launch_arguments=[('params_file', LaunchConfiguration('parameters'))]
        )
    ])

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(launch_microstrain_imu)
    return ld
