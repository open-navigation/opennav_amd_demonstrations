from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_arg_imu_filter = DeclareLaunchArgument(
        'imu_filter',
        default_value='/etc/clearpath/platform/config/imu_filter.yaml',
        description='')

    imu_filter = LaunchConfiguration('imu_filter')

    # Include Packages
    pkg_clearpath_platform = FindPackageShare('clearpath_platform')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_platform, 'launch', 'platform.launch.py'])
    launch_file_diagnostics = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'diagnostics.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [('setup_path', '/etc/clearpath/'),
             ('use_sim_time', 'false'),
             ('namespace', 'j100_0849')]
    )

    launch_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_diagnostics]),
    )

    # Nodes
    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace='j100_0849',
        output='screen',
        parameters=
            [{'hz': 1.0, 'dev': '',
              'connected_topic': 'platform/wifi_connected',
              'connection_topic': 'platform/wifi_status'}],
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_diagnostics',
        namespace='j100_0849',
        output='screen',
        arguments=['-s', '/etc/clearpath/'],
    )

    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_diagnostics',
        namespace='j100_0849',
        output='screen',
        arguments=['-s', '/etc/clearpath/']
    )

    node_imu_filter_node = Node(
        name='imu_filter_node',
        executable='imu_filter_madgwick_node',
        package='imu_filter_madgwick',
        namespace='j100_0849',
        output='screen',
        remappings=
            [('imu/data_raw', 'sensors/imu_0/data_raw'),
             ('imu/mag', 'sensors/imu_0/magnetic_field'),
             ('imu/data', 'sensors/imu_0/data'),
             ('/tf', 'tf')],
        parameters=[imu_filter],
    )

    node_micro_ros_agent = Node(
        name='micro_ros_agent',
        executable='micro_ros_agent',
        package='micro_ros_agent',
        namespace='j100_0849',
        output='screen',
        arguments=['serial', '--dev', '/dev/clearpath/j100'],
    )

    node_nmea_topic_driver = Node(
        name='nmea_topic_driver',
        executable='nmea_topic_driver',
        package='nmea_navsat_driver',
        namespace='j100_0849',
        output='screen',
        remappings=
            [('nmea_sentence', 'sensors/gps_0/nmea_sentence'),
             ('fix', 'sensors/gps_0/fix'),
             ('heading', 'sensors/gps_0/heading'),
             ('time_reference', 'sensors/gps_0/time_reference'),
             ('vel', 'sensors/gps_0/vel')],
    )

    # Processes
    process_configure_mcu = ExecuteProcess(
        shell=True,
        cmd=
            [['export ROS_DOMAIN_ID=0;'],
             [FindExecutable(name='ros2'),
              ' service call platform/mcu/configure',
              ' clearpath_platform_msgs/srv/ConfigureMcu',
              ' "{domain_id: 0,',
              ' robot_namespace: \'j100_0849\'}"'],
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_imu_filter)
    ld.add_action(launch_platform)
    ld.add_action(launch_diagnostics)
    ld.add_action(node_wireless_watcher)
    ld.add_action(node_battery_state_estimator)
    ld.add_action(node_battery_state_control)
    ld.add_action(node_imu_filter_node)
    ld.add_action(node_micro_ros_agent)
    ld.add_action(node_nmea_topic_driver)
    ld.add_action(process_configure_mcu)
    return ld
