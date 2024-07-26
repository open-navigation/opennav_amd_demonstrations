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

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
from launch_ros.actions import LifecycleNode, Node
import lifecycle_msgs


def generate_launch_description():
    honeybee_nav_dir = get_package_share_directory('honeybee_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('params_file')

    params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(honeybee_nav_dir, 'config', '3d_localization.yaml')
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    # Specify map to use in 3d_localization.yaml
    lidar_localization_cmd = LifecycleNode(
        name='lidar_localization',
        package='lidar_localization_ros2',
        namespace='',
        executable='lidar_localization_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/velodyne_points', '/sensors/lidar_0/points'),
                    ('/odom', '/platform/odom/filtered'),
                    ('/imu', '/platform/imu_1/data')],
        output='screen',
        condition=IfCondition(PythonExpression(['not ', slam])),
    )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization_cmd,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg='-- Unconfigured --'),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization_cmd),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization_cmd,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg='-- Inactive --'),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization_cmd),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
    )

    scan_matcher_cmd = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/input_cloud', '/sensors/lidar_0/points'),
                    ('/odom', '/platform/odom/filtered'),
                    ('/imu', '/platform/imu_1/data')],
        output='screen',
        condition=IfCondition(slam),
    )

    graph_slam_cmd = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(slam),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(scan_matcher_cmd)
    ld.add_action(graph_slam_cmd)
    ld.add_action(lidar_localization_cmd)
    ld.add_action(to_inactive)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    return ld
