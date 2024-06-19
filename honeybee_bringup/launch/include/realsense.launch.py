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

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'config', 'realsense.yaml']))

    realsense2_camera_node = Node(
        package='realsense2_camera',
        namespace='sensors',
        name='camera_0',
        executable='realsense2_camera_node',
        parameters=[LaunchConfiguration('parameters')],
        output='screen',
        remappings=[
            # Color
            ('camera_0/color/image_raw', 'camera_0/color/image'),
            ('camera_0/color/image_raw/compressed', 'camera_0/color/compressed'),
            ('camera_0/color/image_raw/compressedDepth', 'camera_0/color/compressedDepth'),
            ('camera_0/color/image_raw/theora', 'camera_0/color/theora'),
            # Depth
            ('camera_0/depth/image_rect_raw', 'camera_0/depth/image'),
            ('camera_0/depth/image_rect_raw/compressed', 'camera_0/depth/compressed'),
            ('camera_0/depth/image_rect_raw/compressedDepth', 'camera_0/depth/compressedDepth'),
            ('camera_0/depth/image_rect_raw/theora', 'camera_0/depth/theora'),
            # Points
            ('camera_0/depth/color/points', 'camera_0/points')
        ]
    )

    static_tf_bridge = Node(
        name='camera_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        output='screen',
        arguments=['--frame-id', 'camera_0_link', '--child-frame-id', 'camera_link'],
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(static_tf_bridge)
    ld.add_action(realsense2_camera_node)
    return ld