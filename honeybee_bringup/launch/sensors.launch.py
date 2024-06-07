from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    bringup_dir = FindPackageShare('honeybee_bringup')

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'launch', 'include', 'realsense.launch.py'])]),
    )

    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'launch', 'include', 'microstrain.launch.py'])]),
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'launch', 'include', 'ouster.launch.py'])]),
    )

    ld = LaunchDescription()
    ld.add_action(launch_camera)
    ld.add_action(launch_imu)
    ld.add_action(launch_lidar)
    return ld
