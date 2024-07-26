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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


# This launch file is for testing ground segmentation with bags offline.
def generate_launch_description():
    bagfile = LaunchConfiguration('bagfile')
    declare_bag_cmd = DeclareLaunchArgument(
        'bagfile',
        default_value='',
        description='Path to bag file to use')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    patchworkpp_node = Node(
        package='patchworkpp',
        executable='patchworkpp_node',
        name='patchworkpp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', '/sensors/lidar_0/points'),
            ('/patchworkpp/nonground', '/sensors/lidar_0/segmented_points'),
            ('/patchworkpp/ground', '/sensors/lidar_0/ground_points')
        ],
        parameters=[
            {'base_frame': 'os0_lidar',
             'use_sim_time': use_sim_time,
             'sensor_height': 0.495,
             'num_iter': 3,
             'num_lpr': 20,
             'num_min_pts': 0,
             'max_range': 20.0,
             'min_range': 0.0,
             'th_dist': 0.125,
             'th_seeds': 0.3,
             'th_seeds_v': 0.25,
             'th_dist_v': 0.1,  # Originally 0.9
             'uprightness_thr': 0.101,
             'verbose': True}
        ],
    )

    bagfile_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bagfile],
        output='screen',
        condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_bag_cmd,
        patchworkpp_node,
        bagfile_play])
