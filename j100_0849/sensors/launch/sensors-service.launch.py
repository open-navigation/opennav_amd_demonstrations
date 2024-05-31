from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages

    # Declare launch files
    launch_file_camera_0 = '/etc/clearpath/sensors/launch/camera_0.launch.py'
    launch_file_imu_1 = '/etc/clearpath/sensors/launch/imu_1.launch.py'

    # Include launch files
    launch_camera_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_0]),
    )

    launch_imu_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_1]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_camera_0)
    ld.add_action(launch_imu_1)
    return ld
