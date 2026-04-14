import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mapping')
    
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'my_config.lua'

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[('/scan', '/autodrive/roboracer_1/lidar'), ('/imu', '/autodrive/roboracer_1/imu'), ('/odom', '/autodrive/roboracer_1/odom')] # Change if your topic is different
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05']
        ),
    ])
