from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_file_camera = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'include', 'realsense.launch.py'])
    launch_file_imu = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'include', 'microstrain.launch.py'])
    launch_file_lidar = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'include', 'ouster.launch.py'])

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera]),
    )

    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu]),
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar]),
    )

    ld = LaunchDescription()
    ld.add_action(launch_camera)
    ld.add_action(launch_imu)
    ld.add_action(launch_lidar)
    return ld
