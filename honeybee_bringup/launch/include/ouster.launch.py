#!/usr/bin/python3
# Copyright 2020, Steve Macenski
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os


def generate_launch_description():
    honeybee_dir = get_package_share_directory('honeybee_bringup')

    # Acquire the driver param file
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(honeybee_dir, 'config', 'ouster.yaml'),
        description='')

    driver_node = LifecycleNode(package='ros2_ouster',
                                executable='ouster_driver',
                                name='ouster_driver',
                                output='screen',
                                namespace='',
                                emulate_tty=True,
                                parameters=[LaunchConfiguration('params_file')])

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(driver_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name='ouster_driver'),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        activate_event,
        configure_event,
        shutdown_event,
    ])
