import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from lidar_waypoint_racing import utils
import csv

class WaypointLoader(Node):
    def __init__(self):
        super().__init__('waypoint_loader')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.waypoints = self.load_waypoints()
        self.timer = self.create_timer(1.0, self.publish_waypoints)

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

    def publish_waypoints(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for waypoint in self.waypoints:
            p = utils.create_point(waypoint[0], waypoint[1], 0.0)
            marker.points.append(p)

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    waypoint_loader = WaypointLoader()
    rclpy.spin(waypoint_loader)
    waypoint_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
