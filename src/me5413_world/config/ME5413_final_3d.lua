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
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_to_tf = false,
  publish_frame_projected_to_2d = false,
  -- Add a hidden parameter 'publish_tracked_pose' for 'evo' plotting
  publish_tracked_pose = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--------------------------------------------------------------------------------
-- This config can not work
--------------------------------------------------------------------------------
-- MAP_BUILDER.use_trajectory_builder_3d = true
-- MAP_BUILDER.num_background_threads = 7
-- -- TRAJECTORY_BUILDER_3D.use_imu_data = true
-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160

-- -- Local SLAM
-- -- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 10

-- -- Global SLAM
-- POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 320
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.min_score = 0.62
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66


--------------------------------------------------------------------------------
-- This config can work
--------------------------------------------------------------------------------
-- MAP_BUILDER.use_trajectory_builder_3d = true
--    TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
--    TRAJECTORY_BUILDER_3D.min_range = 0.3
--   --  defualt 60 
--    TRAJECTORY_BUILDER_3D.max_range = 60. --30
--    TRAJECTORY_BUILDER_2D.min_z = -0.8
--    TRAJECTORY_BUILDER_2D.max_z = 1.0
--    TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true 
--    TRAJECTORY_BUILDER_3D.submaps.num_range_data = 65  --注意和optimize_every_n_nodes对应！ --30 --65
--    TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.01 
   
--    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 0.1
   
   
--    MAP_BUILDER.num_background_threads = 5 
--    POSE_GRAPH.optimization_problem.huber_scale = 5e2
 
 
--    POSE_GRAPH.optimize_every_n_nodes = 130 
 
--    POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  
--    POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 16
--    POSE_GRAPH.constraint_builder.min_score = 0.95
--    POSE_GRAPH.constraint_builder.global_localization_min_score = 0.95
 
--    POSE_GRAPH.optimization_problem.fix_z_in_3d = true
--    POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false
--    POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1
--    POSE_GRAPH.optimization_problem.odometry_translation_weight = 1
--    POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1
--    POSE_GRAPH.max_num_final_iterations = 200


--------------------------------------------------------------------------------
-- Reorginized config
--------------------------------------------------------------------------------
-- MAP_BUILDER.use_trajectory_builder_3d = true
-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_3D.min_range = 0.075  -- defualt 60 
-- TRAJECTORY_BUILDER_3D.max_range = 100.0 
-- TRAJECTORY_BUILDER_3D.min_z = -1.0
-- TRAJECTORY_BUILDER_3D.max_z = 2.0

-- TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100 -- 注意和optimize_every_n_nodes对应！ --30 --65
-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

-- -- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight = 10
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight= 10
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5

-- MAP_BUILDER.num_background_threads = 10


MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAX_3D_RANGE = 100
INTENSITY_THRESHOLD = 40

TRAJECTORY_BUILDER_3D = {
  min_range = 0.075,
  max_range = MAX_3D_RANGE,
  min_z = -1.0,
  max_z = 2.0,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.05,

  -- 在3d slam 时会有2个自适应体素滤波, 一个生成高分辨率点云, 一个生成低分辨率点云
  high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 15.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = MAX_3D_RANGE,
  },

  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    -- 在3D中,occupied_space_weight_0和occupied_space_weight_1参数分别与高分辨率和低分辨率滤波点云相关
    occupied_space_weight_0 = 1.,
    occupied_space_weight_1 = 6.,
    intensity_cost_function_options_0 = {
        weight = 0.5,
        huber_scale = 0.3,
        intensity_threshold = INTENSITY_THRESHOLD,
    },
    translation_weight = 5.,
    rotation_weight = 4e2,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.1,
    max_angle_radians = 0.002,
  },

  rotational_histogram_size = 120,

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,
  pose_extrapolator = {
    use_imu_based = true,
    constant_velocity = {
      imu_gravity_time_constant = 10.,
      pose_queue_duration = 0.001,
    },
    -- TODO(wohe): Tune these parameters on the example datasets.
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {
    -- 2种分辨率的地图
    high_resolution = 0.1,           -- 用于近距离测量的高分辨率hybrid网格 for local SLAM and loop closure, 用来与小尺寸voxel进行精匹配
    high_resolution_max_range = 35.,  -- 在插入 high_resolution map 之前过滤点云的最大范围
    low_resolution = 0.45,
    num_range_data = 50,             -- 用于远距离测量的低分辨率hybrid网格 for local SLAM only, 用来与大尺寸voxel进行粗匹配
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
      num_free_space_voxels = 2,
      intensity_threshold = INTENSITY_THRESHOLD,
    },
  },

  -- When setting use_intensites to true, the intensity_cost_function_options_0
  -- parameter in ceres_scan_matcher has to be set up as well or otherwise
  -- CeresScanMatcher will CHECK-fail.
  use_intensities = false,
}



POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 16
POSE_GRAPH.constraint_builder.min_score = 0.85 --0.95
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9 --0.95
POSE_GRAPH.optimization_problem.fix_z_in_3d = true
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true -- true
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1
POSE_GRAPH.max_num_final_iterations = 200

return options
