#!/bin/bash

# 1. Finish the trajectory
echo "Finishing trajectory..."
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 2. Get the name
echo "Please enter a map name: "
read map_name

# 3. Create directory (Using $HOME to ensure the path is valid)
MAP_DIR="/home/ros2_ws/src/maps/$map_name"
mkdir -p "$MAP_DIR"

echo "Saving map to $MAP_DIR/$map_name.pbstream"

# 4. Write State (Note the unquoted true and fixed path)
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$MAP_DIR/$map_name.pbstream', include_unfinished_submaps: true}"

# 5. Wait a second for the filesystem to catch up
# sleep 2

# 6. Convert to ROS Map
# ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
#   -pbstream_filename "$MAP_DIR/$map_name.pbstream" \
#   -map_filestem "$MAP_DIR/$map_name" \
#   -resolution 0.05
