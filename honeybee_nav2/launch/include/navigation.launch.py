# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    container_name = LaunchConfiguration('container_name')
    localization_type = LaunchConfiguration('localization_type')
    nav2pose_bt_xml = LaunchConfiguration('nav2pose_bt_xml')

    general_lifecycle_nodes = ['controller_server',
                               'smoother_server',
                               'velocity_smoother']

    composite_lifecycle_nodes = ['planner_server',
                                 'behavior_server',
                                 'bt_navigator',
                                 'waypoint_follower']

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': 'True',
        'default_nav_to_pose_bt_xml': nav2pose_bt_xml}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_localization_type_cmd = DeclareLaunchArgument(
        'localization_type',
        default_value='2D',
        description='Whether to use indoor (2D), outdoor (3D), or GPS (GPS) localization',
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'nav2pose_bt_xml',
        default_value=os.path.join(
            bt_navigator_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Which navigate to pose BT to use',
    )

    # General modules all navigation demos require
    load_general_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_general_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': True,
                             'node_names': general_lifecycle_nodes}]),
        ],
    )

    # 2D navigation uses the collision monitor which takes output of velocity smoother
    load_2D_nodes = LoadComposableNodes(
        target_container=container_name,
        condition=IfCondition(PythonExpression(["'", localization_type, "'=='2D'"])),
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[configured_params]),
            ComposableNode(
                package='opennav_docking',
                plugin='opennav_docking::DockingServer',
                name='docking_server',
                parameters=[configured_params]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_application_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': True,
                             'node_names': ['docking_server', 'collision_monitor']}]),
        ],
    )

    # Non-2D navigation doesn't use the collision monitor; remap cmd_vel_smoothed to cmd_vel
    load_non_2D_nodes = LoadComposableNodes(
        target_container=container_name,
        condition=IfCondition(PythonExpression(["'", localization_type, "'!='2D'"])),
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        ],
    )

    # 3D Urban Route Navigation demo doesn't require some modules, so we can skip them in that case
    load_composite_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        condition=IfCondition(PythonExpression(["'", localization_type, "'!='3D'"])),
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params]),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params]),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_composite_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': True,
                             'node_names': composite_lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_localization_type_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(load_general_composable_nodes)
    ld.add_action(load_composite_composable_nodes)
    ld.add_action(load_2D_nodes)
    ld.add_action(load_non_2D_nodes)
    return ld
