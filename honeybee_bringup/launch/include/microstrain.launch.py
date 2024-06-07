from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_microstrain_inertial_driver = FindPackageShare('microstrain_inertial_driver')

    parameters = LaunchConfiguration('parameters')
    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'config', 'microstrain.yaml']))

    launch_microstrain_imu = Node(
        package = 'microstrain_inertial_driver',
        executable = 'microstrain_inertial_driver_node',
        name = 'microstrain_inertial_driver',
        parameters = [parameters],
        output='screen',
        remappings=[
            ('/imu/data', 'data'), ('/moving_ang', 'moving_ang')]
        )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(launch_microstrain_imu)
    return ld
