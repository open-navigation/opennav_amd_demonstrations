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
    description_dir = get_package_share_directory('honeybee_description')

    # Get the appropriate Rviz configuration file
    rviz_config_file = LaunchConfiguration('rviz_config')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(description_dir, 'rviz', 'config.rviz'),
        description='Full path to the RVIZ config file to use')

    # Start the robot state publisher to publish the robot description
    urdf = os.path.join(description_dir, 'urdf', 'honeybee_description.urdf.xacro')
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True,
             'robot_description': ParameterValue(Command(['xacro ', str(urdf)]), value_type=str)}
        ])

    # Launch rviz to visualize the robot
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}])

    # Set transforms the gazebo / ros control will populate in a practical application
    transform_front_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.131', '0.187795', '0.0345', '0', '0', '0',
            'chassis_link', 'front_left_wheel_link']
    )

    transform_front_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.131', '-0.187795', '0.0345', '0', '0', '0',
            'chassis_link', 'front_right_wheel_link']
    )

    transform_rear_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.131', '0.187795', '0.0345', '0', '0', '0',
            'chassis_link', 'rear_left_wheel_link']
    )

    transform_rear_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.131', '-0.187795', '0.0345', '0', '0', '0',
            'chassis_link', 'rear_right_wheel_link']
    )

    # Create the launch description and populate it
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)

    # Launch nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(transform_front_left)
    ld.add_action(transform_front_right)
    ld.add_action(transform_rear_left)
    ld.add_action(transform_rear_right)
    return ld
