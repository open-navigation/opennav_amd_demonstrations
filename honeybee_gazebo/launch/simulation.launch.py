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
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    sim_dir = get_package_share_directory('honeybee_gazebo')
    desc_dir = get_package_share_directory('honeybee_description')
    bringup_dir = get_package_share_directory('honeybee_bringup')
    cpr_dir = get_package_share_directory('clearpath_platform_description')
    realsense_dir = get_package_share_directory('realsense2_camera')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'depot.sdf'),
        description='Full path to world model file to load',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute client'
    )

    declare_jsp_cmd = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='False',
        description='Whether to launch the joint state publisher',
    )

    pose = {
        'x': LaunchConfiguration('x_pose', default='-8.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # The SDF file for the world is a xacro file because we wanted to
    # conditionally load the SceneBroadcaster plugin based on wheter we're
    # running in headless mode. But currently, the Gazebo command line doesn't
    # take SDF strings for worlds, so the output of xacro needs to be saved into
    # a temporary file and passed to Gazebo.
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items())

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(sim_dir, 'worlds'))
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[{
            'config_file': os.path.join(sim_dir, 'config', 'gazebo_bridge.yaml'),
            'use_sim_time': True}],
        output='screen',
    )

    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['/sensors/camera_0/image'],
        remappings=[('/sensors/camera_0/image', '/sensors/camera_0/color/image')])

    camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=['/sensors/camera_0/depth_image'],
        remappings=[('/sensors/camera_0/depth_image', '/sensors/camera_0/depth/image')])

    gz_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'honeybee',
            '-topic', 'robot_description',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        parameters=[{'use_sim_time': True}]
    )

    urdf = os.path.join(desc_dir, 'urdf', 'honeybee_description.urdf.xacro')
    control_config = PathJoinSubstitution([bringup_dir, 'config', 'ros_control.yaml'])
    launch_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True,
             'robot_description': ParameterValue(
              Command(['xacro ', str(urdf), ' ', 'is_sim:=True',
                       ' ', 'gazebo_controllers:=', control_config]), value_type=str)}
        ],
        remappings=[('joint_states', 'platform/joint_states')],
        condition=IfCondition(use_joint_state_publisher)
    )

    pc2_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{'range_max': 25.0, 'angle_increment': 0.01227,  # 512x10 mode
                     'target_frame': 'os0_sensor', 'scan_time': 0.10}],  # Invert the frame
        remappings=[('/cloud_in', '/sensors/lidar_0/points'),
                    ('scan', '/sensors/lidar_0/scan')],
    )

    set_env_vars_resources1 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(desc_dir)).parent.resolve()))
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(sim_dir)).parent.resolve()))
    set_env_vars_resources3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(cpr_dir)).parent.resolve()))
    set_env_vars_resources4 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(realsense_dir)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_jsp_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources1)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(set_env_vars_resources3)
    ld.add_action(set_env_vars_resources4)
    ld.add_action(start_rviz_cmd)

    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(gz_robot)
    ld.add_action(gz_bridge)
    ld.add_action(camera_bridge_image)
    ld.add_action(camera_bridge_depth)

    ld.add_action(launch_robot_state_publisher_cmd)
    ld.add_action(pc2_to_laserscan_cmd)

    return ld
