# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def convert_value(value):
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        if value.lower() == 'true':
            return True
        elif value.lower() == 'false':
            return False
    return value


def load_parameters(context, args):
    default_params = {arg.name: LaunchConfiguration(arg.name).perform(context) for arg in args}
    return {
        key: (convert_value(value))
        for key, value in default_params.items()
    }


def generate_launch_description():
    args = [
        DeclareLaunchArgument('depth_registration', default_value='true'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('usb_port', default_value=''),
        DeclareLaunchArgument('device_num', default_value='1'),
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),  # SENSOR_DATA
        DeclareLaunchArgument('enable_point_cloud', default_value='True'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='false'),
        DeclareLaunchArgument('connection_delay', default_value='10'),
        DeclareLaunchArgument('color_width', default_value='424'),
        DeclareLaunchArgument('color_height', default_value='240'),
        DeclareLaunchArgument('color_fps', default_value='6'),
        DeclareLaunchArgument('color_format', default_value='ANY'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('color_qos', default_value='default'),
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_color_auto_exposure', default_value='true'),
        DeclareLaunchArgument('color_exposure', default_value='-1'),
        DeclareLaunchArgument('color_gain', default_value='-1'),
        DeclareLaunchArgument('enable_color_auto_white_balance', default_value='true'),
        DeclareLaunchArgument('color_white_balance', default_value='-1'),
        DeclareLaunchArgument('depth_width', default_value='424'),
        DeclareLaunchArgument('depth_height', default_value='240'),
        DeclareLaunchArgument('depth_fps', default_value='6'),
        DeclareLaunchArgument('depth_format', default_value='ANY'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('depth_qos', default_value='default'),
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_width', default_value='0'),
        DeclareLaunchArgument('left_ir_height', default_value='0'),
        DeclareLaunchArgument('left_ir_fps', default_value='0'),
        DeclareLaunchArgument('left_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_left_ir', default_value='false'),
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_width', default_value='0'),
        DeclareLaunchArgument('right_ir_height', default_value='0'),
        DeclareLaunchArgument('right_ir_fps', default_value='0'),
        DeclareLaunchArgument('right_ir_format', default_value='ANY'),
        DeclareLaunchArgument('enable_right_ir', default_value='false'),
        DeclareLaunchArgument('right_ir_qos', default_value='default'),
        DeclareLaunchArgument('right_ir_camera_info_qos', default_value='default'),
        DeclareLaunchArgument('enable_ir_auto_exposure', default_value='true'),
        DeclareLaunchArgument('ir_exposure', default_value='-1'),
        DeclareLaunchArgument('ir_gain', default_value='-1'),
        DeclareLaunchArgument('enable_sync_output_accel_gyro', default_value='false'),
        DeclareLaunchArgument('enable_accel', default_value='false'),
        DeclareLaunchArgument('accel_rate', default_value='200hz'),
        DeclareLaunchArgument('accel_range', default_value='4g'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('gyro_rate', default_value='200hz'),
        DeclareLaunchArgument('gyro_range', default_value='1000dps'),
        DeclareLaunchArgument('liner_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('angular_vel_cov', default_value='0.01'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0'),
        DeclareLaunchArgument('ir_info_url', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=''),
        DeclareLaunchArgument('log_level', default_value='none'),
        DeclareLaunchArgument('enable_publish_extrinsic', default_value='false'),
        DeclareLaunchArgument('enable_d2c_viewer', default_value='false'),
        DeclareLaunchArgument('enable_ldp', default_value='true'),
        DeclareLaunchArgument('enable_soft_filter', default_value='true'),
        DeclareLaunchArgument('soft_filter_max_diff', default_value='-1'),
        DeclareLaunchArgument('soft_filter_speckle_size', default_value='-1'),
        DeclareLaunchArgument('sync_mode', default_value='standalone'),
        DeclareLaunchArgument('depth_delay_us', default_value='0'),
        DeclareLaunchArgument('color_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger2image_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_delay_us', default_value='0'),
        DeclareLaunchArgument('trigger_out_enabled', default_value='true'),
        DeclareLaunchArgument('frames_per_trigger', default_value='2'),
        DeclareLaunchArgument('software_trigger_period', default_value='33'),  # ms
        DeclareLaunchArgument('enable_frame_sync', default_value='true'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        DeclareLaunchArgument('use_hardware_time', default_value='true'),
        DeclareLaunchArgument('enable_depth_scale', default_value='true'),
        DeclareLaunchArgument('enable_decimation_filter', default_value='True'),
        DeclareLaunchArgument('enable_hdr_merge', default_value='false'),
        DeclareLaunchArgument('enable_sequence_id_filter', default_value='false'),
        DeclareLaunchArgument('enable_threshold_filter', default_value='false'),
        DeclareLaunchArgument('enable_noise_removal_filter', default_value='true'),
        DeclareLaunchArgument('enable_spatial_filter', default_value='false'),
        DeclareLaunchArgument('enable_temporal_filter', default_value='false'),
        DeclareLaunchArgument('enable_hole_filling_filter', default_value='false'),
        DeclareLaunchArgument('decimation_filter_scale_', default_value='6'),
        DeclareLaunchArgument('sequence_id_filter_id', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_max', default_value='-1'),
        DeclareLaunchArgument('threshold_filter_min', default_value='-1'),
        DeclareLaunchArgument('noise_removal_filter_min_diff', default_value='256'),
        DeclareLaunchArgument('noise_removal_filter_max_size', default_value='80'),
        DeclareLaunchArgument('spatial_filter_alpha', default_value='-1.0'),
        DeclareLaunchArgument('spatial_filter_diff_threshold', default_value='-1'),
        DeclareLaunchArgument('spatial_filter_magnitude', default_value='-1'),
        DeclareLaunchArgument('spatial_filter_radius', default_value='-1'),
        DeclareLaunchArgument('temporal_filter_diff_threshold', default_value='-1.0'),
        DeclareLaunchArgument('temporal_filter_weight', default_value='-1.0'),
        DeclareLaunchArgument('hole_filling_filter_mode', default_value=''),
        DeclareLaunchArgument('hdr_merge_exposure_1', default_value='-1'),
        DeclareLaunchArgument('hdr_merge_gain_1', default_value='-1'),
        DeclareLaunchArgument('hdr_merge_exposure_2', default_value='-1'),
        DeclareLaunchArgument('hdr_merge_gain_2', default_value='-1'),
        DeclareLaunchArgument('align_mode', default_value='SW'),
        DeclareLaunchArgument('diagnostic_period', default_value='1.0'),
        DeclareLaunchArgument('enable_laser', default_value='true'),
        DeclareLaunchArgument('depth_precision', default_value=''),
        DeclareLaunchArgument('device_preset', default_value='Default'),
        DeclareLaunchArgument('laser_on_off_mode', default_value='0'),
        DeclareLaunchArgument('retry_on_usb3_detection_failure', default_value='false'),
        DeclareLaunchArgument('laser_energy_level', default_value='-1'),
        DeclareLaunchArgument('enable_3d_reconstruction_mode', default_value='false'),
        DeclareLaunchArgument('enable_sync_host_time', default_value='true'),
        DeclareLaunchArgument('time_domain', default_value='device'),
        DeclareLaunchArgument('enable_color_undistortion', default_value='false'),
        DeclareLaunchArgument('config_file_path', default_value=''),
        DeclareLaunchArgument('enable_heartbeat', default_value='false'),
    ]

    static_tf_bridge = Node(
        name='camera_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        output='screen',
        arguments=['--frame-id', 'camera_0_link', '--child-frame-id', 'camera_link'],
    )

    def get_params(context, args):
        return [load_parameters(context, args)]

    def create_node_action(context, args):
        params = get_params(context, args)
        return [
            ComposableNodeContainer(
                name='camera_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='orbbec_camera',
                        plugin='orbbec_camera::OBCameraNodeDriver',
                        name='camera_0',
                        namespace='sensors/camera_0',
                        parameters=params,
                        remappings=[
                            # Color
                            ('color/image_raw', '/sensors/camera_0/color/image'),
                            ('color/image_raw/compressed', '/sensors/camera_0/color/compressed'),
                            ('color/image_raw/compressedDepth', '/sensors/camera_0/color/compressedDepth'),
                            ('color/image_raw/theora', '/sensors/camera_0/color/theora'),
                            # Depth
                            ('depth/image_raw', '/sensors/camera_0/depth/image'),
                            ('depth/image_raw/compressed', '/sensors/camera_0/depth/compressed'),
                            ('depth/image_raw/compressedDepth', '/sensors/camera_0/depth/compressedDepth'),
                            ('depth/image_raw/theora', '/sensors/camera_0/depth/theora'),
                            # Points
                            ('depth/points', '/sensors/camera_0/points')
                        ]
                    ),
                ],
                output='screen',
            )
        ]

    return LaunchDescription(
        [static_tf_bridge] + args + [
            OpaqueFunction(function=lambda context: create_node_action(context, args))
        ]
    )
