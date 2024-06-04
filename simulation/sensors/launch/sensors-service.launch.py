from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='/world/warehouse/model/robot/link/base_link/sensor/',
        description='Ignition sensor topic prefix')

    prefix = LaunchConfiguration('prefix')

    # Include Packages

    # Declare launch files
    launch_file_camera_0 = '/home/lcamero/open_navigation/simulation/sensors/launch/camera_0.launch.py'
    launch_file_imu_1 = '/home/lcamero/open_navigation/simulation/sensors/launch/imu_1.launch.py'

    # Include launch files
    launch_camera_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_camera_0]),
    )

    launch_imu_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_1]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(launch_camera_0)
    ld.add_action(launch_imu_1)
    return ld
