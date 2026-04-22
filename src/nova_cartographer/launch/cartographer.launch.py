import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nova_cartographer')
    
    return LaunchDescription([
        # 1. Start Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', os.path.join(pkg_share, 'config'),
                '-configuration_basename', 'nova.lua'
            ],
            remappings=[
                ('/odom', '/autodrive/roboracer_1/odom'),
                ('/scan', '/autodrive/roboracer_1/lidar'),
                ('/imu', '/autodrive/roboracer_1/imu'),
            ],
        ),
        # 2. Start Occupancy Grid Node (converts SLAM submaps to a 2D Map)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05'] # 5cm resolution
        ),
    ])
