#!/usr/bin/python3
# Copyright 2024, Open Navigation LLC
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

"""Launch a sensor node and processing nodes."""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    honeybee_dir = get_package_share_directory('honeybee_bringup')
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(honeybee_dir, 'config', 'ouster.yaml'),
        description='')

    os_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_sensor',
        namespace='sensors/lidar_0',
        parameters=[params_file]
    )

    os_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace='sensors/lidar_0',
        parameters=[params_file]
    )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace='sensors/lidar_0',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
        ],
        output='screen',
    )

    # Obtain a laser scan representative of the entire useful band the robot might experience
    pc2_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{'min_height': -0.2032, 'max_height': 0.127,  # -8 to +5 inches in sensor frame
                     'range_max': 25.0, 'angle_increment': 0.01227,  # 512x10 mode
                     'target_frame': 'os0_sensor', 'scan_time': 0.10}],  # Invert the frame
        remappings=[('/cloud_in', '/sensors/lidar_0/points'),
                    ('scan', '/sensors/lidar_0/scan')],
    )

    # Obtain segmented pointclouds for ground and non-ground points, after sensor is up
    patchworkpp_cmd = Node(
        package='patchworkpp',
        executable='patchworkpp_node',
        name='patchworkpp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', '/sensors/lidar_0/points'),
            ('/patchworkpp/nonground', '/sensors/lidar_0/segmented_points'),
            ('/patchworkpp/ground', '/sensors/lidar_0/ground_points')
        ],
        parameters=[params_file],
    )

    # Bringup since Configure is flaky in the driver, we need a retry mechanism
    activation_watchdog_cmd = Node(
        package='honeybee_watchdogs',
        executable='ouster_activation_watchdog',
        name='ouster_activation_watchdog'
    )

    return launch.LaunchDescription([
        params_file_arg,
        os_container,
        pc2_to_laserscan_cmd,
        activation_watchdog_cmd,
        TimerAction(period=0.05, actions=[patchworkpp_cmd])
    ])
