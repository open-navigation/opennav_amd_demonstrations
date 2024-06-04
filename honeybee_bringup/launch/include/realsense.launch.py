from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'config', 'realsense.yaml']))

    realsense2_camera_node = Node(
        package='realsense2_camera',
        name='intel_realsense',
        executable='realsense2_camera_node',
        parameters=[parameters],
        output='screen',
        remappings=[('/tf_static', 'tf_static'), ('/tf', 'tf')]
    )

    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(realsense2_camera_node)
    return ld
