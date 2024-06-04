from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    launch_file_camera = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'include', 'realsense.launch.py'])
    launch_file_imu = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'include', 'microstrain.launch.py'])
    launch_file_lidar = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'include', 'ouster.launch.py'])

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera]),
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu]),
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar]),
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(launch_camera)
    ld.add_action(launch_imu)
    ld.add_action(launch_lidar)
    return ld
