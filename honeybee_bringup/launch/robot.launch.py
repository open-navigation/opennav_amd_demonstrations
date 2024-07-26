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
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_dir = get_package_share_directory('honeybee_description')
    bringup_dir = FindPackageShare('honeybee_bringup')
    sim_dir = FindPackageShare('honeybee_gazebo')

    use_simulation = LaunchConfiguration('use_simulation')
    use_sim_time = use_simulation
    arg_use_simulation = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Use simulation or hardware'
    )

    launch_robot_hardware = PathJoinSubstitution([
          bringup_dir, 'launch', 'base.launch.py'])
    launch_robot_sensors = PathJoinSubstitution([
          bringup_dir, 'launch', 'sensors.launch.py'])
    launch_robot_simulation = PathJoinSubstitution([
          sim_dir, 'launch', 'simulation.launch.py'])

    launch_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_hardware]),
        launch_arguments=[('use_sim_time', use_sim_time),
                          ('use_simulation', use_simulation)]
    )

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_sensors]),
        launch_arguments=[('use_sim_time', use_sim_time)],
        condition=UnlessCondition(use_simulation)
    )

    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_simulation]),
        launch_arguments=[('use_sim_time', use_sim_time)],
        condition=IfCondition(use_simulation)
    )

    urdf = os.path.join(description_dir, 'urdf', 'honeybee_description.urdf.xacro')
    control_config = PathJoinSubstitution([bringup_dir, 'config', 'ros_control.yaml'])
    launch_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'robot_description': ParameterValue(
                Command(['xacro ', str(urdf), ' ', 'is_sim:=', use_sim_time,
                         ' ', 'gazebo_controllers:=', control_config]), value_type=str)}
        ],
        remappings=[('joint_states', 'platform/joint_states')]
    )

    ld = LaunchDescription()
    ld.add_action(arg_use_simulation)
    ld.add_action(launch_robot_state_publisher_cmd)
    ld.add_action(launch_base)
    ld.add_action(launch_sensors)
    ld.add_action(launch_simulation)
    return ld
