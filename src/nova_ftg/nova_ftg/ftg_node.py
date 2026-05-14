from .ftg import FTG_Controller
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower_node')
        self.declare_parameter('safety_radius', 5)
        self.declare_parameter('max_lidar_dist', 10)
        self.declare_parameter('max_speed', 5)
        self.declare_parameter('range_offset', 270)
        self.declare_parameter('track_width', 4)
        self.ftg = FTG_Controller(mapping=False, debug=True, safety_radius=5, max_lidar_dist=10, max_speed=5, range_offset=270, track_width=4)
        # TODO: remove in prod
        self.add_on_set_parameters_callback(self.parameter_callback)
        
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

    def parameter_callback(self, params):
            for param in params:
                if param.name == 'safety_radius':
                    self.ftg.SAFETY_RADIUS = param.value
                if param.name == 'max_lidar_dist':
                    self.ftg.MAX_LIDAR_DIST = param.value
                if param.name == 'max_speed':
                    self.ftg.MAX_SPEED = param.value
                if param.name == 'range_offset':
                    self.ftg.range_offset = param.value
                if param.name == 'track_width':
                    self.ftg.track_width = param.value
            return SetParametersResult(successful=True)

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
