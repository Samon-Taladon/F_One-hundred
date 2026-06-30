import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from lidar_waypoint_racing import utils
import csv
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.waypoints = self.load_waypoints()
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('target_speed', 2.0)

    def load_waypoints(self):
        waypoints = []
        try:
            with open('waypoints.csv', 'r') as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    waypoints.append((float(row[0]), float(row[1])))
        except FileNotFoundError:
            self.get_logger().error('waypoints.csv not found!')
        return waypoints

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        current_position = (position.x, position.y)
        current_yaw = utils.euler_from_quaternion(orientation)[2]

        if not self.waypoints:
            return

        target_waypoint_index = self.find_target_waypoint(current_position)
        target_waypoint = self.waypoints[target_waypoint_index]

        steering_angle = self.calculate_steering_angle(current_position, current_yaw, target_waypoint)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.get_parameter('target_speed').value
        self.publisher_.publish(drive_msg)

    def find_target_waypoint(self, current_position):
        lookahead_distance = self.get_parameter('lookahead_distance').value
        nearest_waypoint_index = 0
        nearest_distance = float('inf')

        for i, waypoint in enumerate(self.waypoints):
            distance = utils.distance(current_position, waypoint)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_waypoint_index = i

        target_waypoint_index = nearest_waypoint_index
        while True:
            target_waypoint_index = (target_waypoint_index + 1) % len(self.waypoints)
            distance_to_target = utils.distance(current_position, self.waypoints[target_waypoint_index])
            if distance_to_target > lookahead_distance:
                break

        return target_waypoint_index

    def calculate_steering_angle(self, current_position, current_yaw, target_waypoint):
        alpha = math.atan2(target_waypoint[1] - current_position[1], target_waypoint[0] - current_position[0]) - current_yaw
        lookahead_distance = utils.distance(current_position, target_waypoint)
        steering_angle = math.atan2(2.0 * 1.0 * math.sin(alpha), lookahead_distance)
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
