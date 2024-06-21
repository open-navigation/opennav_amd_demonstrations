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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    imu_filter_params = PathJoinSubstitution([
        FindPackageShare('honeybee_bringup'), 'config', 'imu_filter.yaml'])
    robot_localization_params = PathJoinSubstitution([
        FindPackageShare('honeybee_bringup'), 'config', 'robot_localization.yaml'])
    teleop_joy_params = PathJoinSubstitution([
        FindPackageShare('honeybee_bringup'), 'config', 'teleop_joy.yaml'])
    twist_mux_params = PathJoinSubstitution([
        FindPackageShare('honeybee_bringup'), 'config', 'twist_mux.yaml'])
    ros_control_params = PathJoinSubstitution([
        FindPackageShare('honeybee_bringup'), 'config', 'ros_control.yaml'])

    setup_path = PathJoinSubstitution([
        FindPackageShare('honeybee_bringup'), 'config', 'include'])

    use_simulation = LaunchConfiguration('use_simulation')
    use_sim_time = use_simulation
    arg_use_simulation = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Use simulation or hardware'
    )

    # Nodes and launch files for robot base
    action_control_group = GroupAction([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[ros_control_params, {'use_sim_time': use_sim_time}],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            remappings=[
              ('platform_velocity_controller/odom', 'platform/odom'),
              ('platform_velocity_controller/cmd_vel_unstamped', 'platform/cmd_vel_unstamped'),
              ('joint_states', 'platform/joint_states'),
              ('dynamic_joint_states', 'platform/dynamic_joint_states'),
              ('~/robot_description', 'robot_description')
            ],
            condition=UnlessCondition(use_sim_time)
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['--controller-manager-timeout', '60', 'joint_state_broadcaster'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['--controller-manager-timeout', '60', 'platform_velocity_controller'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    ])

    node_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[robot_localization_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'platform/odom/filtered')]
        )

    node_joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen',
        name='joy_node',
        parameters=[
            teleop_joy_params,
            {'use_sim_time': use_sim_time}],
        remappings=[
            ('joy', 'joy_teleop/joy'),
            ('joy/set_feedback', 'joy_teleop/joy/set_feedback'),
        ]
    )

    node_teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[
            teleop_joy_params,
            {'use_sim_time': use_sim_time}],
        remappings=[
            ('joy', 'joy_teleop/joy'),
            ('cmd_vel', 'joy_teleop/cmd_vel'),
        ]
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={('cmd_vel_out', 'platform/cmd_vel_unstamped')},
        parameters=[
            twist_mux_params,
            {'use_sim_time': use_sim_time}]
    )

    node_imu_filter_node = Node(
        name='imu_filter_node',
        executable='imu_filter_madgwick_node',
        package='imu_filter_madgwick',
        output='screen',
        remappings=
            [('imu/data_raw', 'sensors/imu_0/data_raw'),
             ('imu/mag', 'sensors/imu_0/magnetic_field'),
             ('imu/data', 'sensors/imu_0/data')],
        parameters=[imu_filter_params, {'use_sim_time': use_sim_time}],
    )

    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        output='screen',
        parameters=
            [{'hz': 1.0, 'dev': '',
              'connected_topic': 'platform/wifi_connected',
              'connection_topic': 'platform/wifi_status',
              'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_simulation)
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_diagnostics',
        output='screen',
        arguments=['-s', setup_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_simulation)
    )

    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_diagnostics',
        output='screen',
        arguments=['-s', setup_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_simulation)
    )

    node_micro_ros_agent = Node(
        name='micro_ros_agent',
        executable='micro_ros_agent',
        package='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/clearpath/j100'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_simulation)
    )

    node_nmea_topic_driver = Node(
        name='nmea_topic_driver',
        executable='nmea_topic_driver',
        package='nmea_navsat_driver',
        output='screen',
        remappings=
            [('nmea_sentence', 'sensors/gps_0/nmea_sentence'),
             ('fix', 'sensors/gps_0/fix'),
             ('heading', 'sensors/gps_0/heading'),
             ('time_reference', 'sensors/gps_0/time_reference'),
             ('vel', 'sensors/gps_0/vel')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_simulation)
    )

    # Defaults to main namespace, no need for empty namespace
    # process_configure_mcu = ExecuteProcess(
    #     shell=True,
    #     cmd=
    #         [['export ROS_DOMAIN_ID=0;'],
    #          [FindExecutable(name='ros2'),
    #           ' service call platform/mcu/configure',
    #           ' clearpath_platform_msgs/srv/ConfigureMcu',
    #           ' "{domain_id: 0,',
    #           ' robot_namespace: \'\'}"'],
    #         ],
    #     condition=UnlessCondition(use_simulation)
    # )

    # Takes alot of CPU and not currently in use
    # launch_diagnostics = GroupAction([
    #     Node(
    #         package='diagnostic_aggregator',
    #         executable='aggregator_node',
    #         output='screen',
    #         parameters=[PathJoinSubstitution([
    #             get_package_share_directory('clearpath_diagnostics'), 'config', 'diagnostics.yaml'])],
    #         condition=UnlessCondition(use_simulation)),
    #     Node(
    #         package='clearpath_diagnostics',
    #         executable='diagnostics_updater',
    #         output='screen',
    #         arguments=['-s', setup_path],
    #         condition=UnlessCondition(use_simulation))
    # ])

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(arg_use_simulation)
    ld.add_action(node_localization)
    ld.add_action(action_control_group) 
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_twist_mux)
    ld.add_action(node_wireless_watcher)
    ld.add_action(node_battery_state_estimator)
    ld.add_action(node_battery_state_control)
    ld.add_action(node_imu_filter_node)
    ld.add_action(node_micro_ros_agent)
    ld.add_action(node_nmea_topic_driver)
    # ld.add_action(launch_diagnostics)
    # ld.add_action(process_configure_mcu)
    return ld
