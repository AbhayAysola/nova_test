from .ftg import FTG_Controller
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower_node')
        self.ftg = FTG_Controller(mapping=False, debug=True, safety_radius=5, max_lidar_dist=10, max_speed=20, range_offset=270, track_width=4)
        
        # Subscriber to the LiDAR topic
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.lidar_callback,
            10) # QoS profile depth
            
        # Publisher to the drive topic (consumed by your PID node)
        self.drive_pub = self.create_publisher(
            AckermannDrive,
            '/drive',
            10)

    def lidar_callback(self, scan_msg):
        speed, angle = self.ftg.process_lidar(scan_msg.ranges)
        drive_msg = AckermannDrive()
        drive_msg.speed = speed
        drive_msg.steering_angle = angle
        drive_msg.acceleration = 0.0
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
