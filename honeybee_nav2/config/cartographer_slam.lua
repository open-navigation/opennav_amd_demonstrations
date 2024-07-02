-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_1_link",
  published_frame = "odom", -- base_link
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  --publish_to_tf: true/false
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html?highlight=use_online_correlative_scan_matching
--https://wilselby.com/2019/06/ouster-os-1-lidar-and-google-cartographer-integration/
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.min_range = 1
TRAJECTORY_BUILDER_2D.max_range = 40
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.25
--TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length
--TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points

POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = .1
