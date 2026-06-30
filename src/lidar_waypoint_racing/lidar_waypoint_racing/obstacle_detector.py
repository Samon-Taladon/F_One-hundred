import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'obstacle_warning', 10)
        self.declare_parameter('min_obstacle_distance', 0.5)

    def scan_callback(self, msg):
        # Obstacle detection in a cone in front of the vehicle
        angle_min = -np.pi / 4  # -45 degrees
        angle_max = np.pi / 4   # +45 degrees
        start_index = int((angle_min - msg.angle_min) / msg.angle_increment)
        end_index = int((angle_max - msg.angle_min) / msg.angle_increment)

        ranges = np.array(msg.ranges[start_index:end_index])
        min_distance = np.min(ranges[np.isfinite(ranges)])

        if min_distance < self.get_parameter('min_obstacle_distance').value:
            warning_msg = Float32()
            warning_msg.data = min_distance
            self.publisher_.publish(warning_msg)
            self.get_logger().warning(f'Obstacle detected at {min_distance} meters!')

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
