from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    description_dir = get_package_share_directory('honeybee_description')
    bringup_dir = get_package_share_directory('honeybee_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='false',
        description='Use simulation time'
    )

    launch_robot_hardware = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'base.launch.py'])
    launch_robot_sensors = PathJoinSubstitution([
          FindPackageShare('honeybee_bringup'), 'launch', 'sensors.launch.py'])

    # TODO if sim, use sim launch file. If hardware, use hardware launch file
    # TODO launch sim + bridge
    launch_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_hardware]),
        launch_arguments=[('use_sim_time', use_sim_time),
    )

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_robot_sensors]),
        launch_arguments=[('use_sim_time', use_sim_time),
    )

    urdf = os.path.join(description_dir, 'urdf', 'honeybee_description.urdf.xacro')
    control_config = os.path.join(bringup_dir, 'config', 'ros_control.yaml')
    launch_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
              Command(['xacro ', str(urdf)], ' ', 'is_sim:=', use_sim_time,
              ' ', 'gazebo_controllers:=', control_config), value_type=str)}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('joint_states', 'platform/joint_states')]
    )

    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(launch_robot_state_publisher_cmd)
    ld.add_action(launch_base)
    ld.add_action(launch_sensors)
    return ld
