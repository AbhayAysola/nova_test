include("map_builder.lua")
include("trajectory_builder.lua")

options = {
	map_builder = MAP_BUILDER,
	trajectory_builder = TRAJECTORY_BUILDER,
	map_frame = "world",
	tracking_frame = "imu", -- The frame Cartographer follows
	published_frame = "roboracer_1", -- The frame it provides TFs for
	odom_frame = "odom",
	provide_odom_frame = false, -- Set to false if you already have an EKF/Odom node
	publish_frame_projected_to_2d = true,
	use_odometry = true,
	use_nav_sat = false,
	use_landmarks = false,
	num_laser_scans = 1,
	num_multi_echo_laser_scans = 0,
	num_subdivisions_per_laser_scan = 1,
	num_point_clouds = 0,
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

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

return options
