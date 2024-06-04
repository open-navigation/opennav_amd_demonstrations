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

    # Nodes
    node_camera_0_gz_bridge = Node(
        name='camera_0_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='j100_0842/sensors/',
        output='screen',
        arguments=
            [
                [
                    prefix
                    ,
                    'camera_0/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                    ,
                ]
                ,
                [
                    prefix
                    ,
                    'camera_0/image@sensor_msgs/msg/Image[ignition.msgs.Image'
                    ,
                ]
                ,
                [
                    prefix
                    ,
                    'camera_0/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                    ,
                ]
                ,
                [
                    prefix
                    ,
                    'camera_0/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'
                    ,
                ]
                ,
            ]
        ,
        remappings=
            [
                (
                    [
                        prefix
                        ,
                        'camera_0/camera_info'
                        ,
                    ]
                    ,
                    'camera_0/color/camera_info'
                )
                ,
                (
                    [
                        prefix
                        ,
                        'camera_0/image'
                        ,
                    ]
                    ,
                    'camera_0/color/image'
                )
                ,
                (
                    [
                        prefix
                        ,
                        'camera_0/points'
                        ,
                    ]
                    ,
                    'camera_0/points'
                )
                ,
                (
                    [
                        prefix
                        ,
                        'camera_0/depth_image'
                        ,
                    ]
                    ,
                    'camera_0/depth/image'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_camera_0_static_tf = Node(
        name='camera_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='j100_0842',
        output='screen',
        arguments=
            [
                '--frame-id'
                ,
                'camera_0_link'
                ,
                '--child-frame-id'
                ,
                'j100_0842/robot/base_link/camera_0'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/tf'
                    ,
                    'tf'
                )
                ,
                (
                    '/tf_static'
                    ,
                    'tf_static'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_camera_0_gz_bridge)
    ld.add_action(node_camera_0_static_tf)
    return ld
