# #!/usr/bin/python3
# # Copyright 2024, Open Navigation LLC
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

"""Launch a sensor node along with os_cloud and os_"""

import os
from pathlib import Path
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
                            ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition


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

    # os_image = ComposableNode(
    #     package='ouster_ros',
    #     plugin='ouster_ros::OusterImage',
    #     name='os_image',
    #     namespace='sensors',
    #     parameters=[params_file]
    # )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace='sensors/lidar_0',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_sensor,
            os_cloud,
            # os_image
        ],
        output='screen',
    )

    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set ',
                  'sensors/lidar_0', '/', node_name, ' ', verb]],
            shell=True)

    sensor_configure_cmd = invoke_lifecycle_cmd('os_sensor', 'configure')
    sensor_activate_cmd = invoke_lifecycle_cmd('os_sensor', 'activate')

    return launch.LaunchDescription([
        params_file_arg,
        os_container,
        sensor_configure_cmd,
        TimerAction(period=1.0, actions=[sensor_activate_cmd])
    ])
