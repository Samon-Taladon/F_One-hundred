#!/usr/bin/env python3
import csv
import os

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class WaypointPathVisualizer(Node):
    """Publish raw waypoint and raceline CSV files as RViz markers."""

    def __init__(self):
        super().__init__('waypoint_path_visualizer')

        self.declare_parameter('raw_path_file', '')
        self.declare_parameter('raceline_path_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('line_width', 0.04)
        self.declare_parameter('point_size', 0.07)

        self.raw_path_file = os.path.expanduser(
            str(self.get_parameter('raw_path_file').value)
        )
        self.raceline_path_file = os.path.expanduser(
            str(self.get_parameter('raceline_path_file').value)
        )
        self.map_frame = str(self.get_parameter('map_frame').value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, '/race_path_markers', qos)

        publish_rate = max(float(self.get_parameter('publish_rate').value), 0.1)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_markers)
        self.paths = {
            'raw_path': self.load_path(self.raw_path_file),
            'raceline': self.load_path(self.raceline_path_file),
        }

        self.get_logger().info(
            'Loaded path markers: '
            f'raw_path={len(self.paths["raw_path"])} points, '
            f'raceline={len(self.paths["raceline"])} points'
        )
        self.publish_markers()

    def load_path(self, path_file):
        points = []
        if not path_file:
            return points
        if not os.path.exists(path_file):
            self.get_logger().warn(f'Waypoint CSV does not exist: {path_file}')
            return points

        with open(path_file, newline='', encoding='utf-8') as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames and 'x' in reader.fieldnames and 'y' in reader.fieldnames:
                for row in reader:
                    try:
                        points.append((float(row['x']), float(row['y'])))
                    except (KeyError, TypeError, ValueError):
                        continue
                return points

        with open(path_file, newline='', encoding='utf-8') as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                try:
                    points.append((float(row[0]), float(row[1])))
                except (TypeError, ValueError, IndexError):
                    continue
        return points

    def publish_markers(self):
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())
        markers.markers.extend(
            self.make_path_markers(
                marker_id=1,
                namespace='raw_path',
                points=self.paths['raw_path'],
                color=(1.0, 0.7, 0.0, 1.0),
                z_offset=0.03,
            )
        )
        markers.markers.extend(
            self.make_path_markers(
                marker_id=10,
                namespace='raceline',
                points=self.paths['raceline'],
                color=(0.0, 0.9, 0.2, 1.0),
                z_offset=0.06,
            )
        )
        self.marker_pub.publish(markers)

    def delete_all_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def make_path_markers(self, marker_id, namespace, points, color, z_offset):
        if not points:
            return []

        line_marker = self.base_marker(marker_id, namespace, Marker.LINE_STRIP)
        line_marker.scale.x = float(self.get_parameter('line_width').value)
        self.set_color(line_marker, color)
        line_marker.points = [
            self.make_point(x, y, z_offset)
            for x, y in points
        ]

        point_marker = self.base_marker(marker_id + 1, namespace, Marker.SPHERE_LIST)
        size = float(self.get_parameter('point_size').value)
        point_marker.scale.x = size
        point_marker.scale.y = size
        point_marker.scale.z = size
        self.set_color(point_marker, color)
        point_marker.points = [
            self.make_point(x, y, z_offset + 0.01)
            for x, y in points
        ]
        return [line_marker, point_marker]

    def base_marker(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def make_point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    @staticmethod
    def set_color(marker, color):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPathVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
