import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    description_dir = get_package_share_directory('honeybee_description')
    bringup_dir = FindPackageShare('honeybee_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    use_simulation = LaunchConfiguration('use_simulation')
    arg_use_simulation = DeclareLaunchArgument(
        'use_simulation',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation'
    )

    launch_robot_hardware = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'base.launch.py'])
    launch_robot_sensors = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'sensors.launch.py'])
    launch_robot_simulation = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'simulation.launch.py'])

    launch_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_hardware]),
        launch_arguments=[('use_sim_time', use_sim_time),
                          ('use_simulation', use_simulation)]
    )

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_sensors]),
        launch_arguments=[('use_sim_time', use_sim_time)],
        condition=UnlessCondition(use_simulation)
    )

    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_simulation]),
        launch_arguments=[('use_sim_time', use_sim_time)],
        condition=IfCondition(use_simulation)
    )

    urdf = os.path.join(description_dir, 'urdf', 'honeybee_description.urdf.xacro')
    control_config = PathJoinSubstitution([bringup_dir, 'config', 'ros_control.yaml'])
    launch_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
              Command(['xacro ', str(urdf), ' ', 'is_sim:=', use_sim_time,
              ' ', 'gazebo_controllers:=', control_config]), value_type=str)}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joint_states', 'platform/joint_states')]
    )

    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_use_simulation)
    ld.add_action(launch_robot_state_publisher_cmd)
    ld.add_action(launch_base)
    ld.add_action(launch_sensors)
    ld.add_action(launch_simulation)
    return ld
