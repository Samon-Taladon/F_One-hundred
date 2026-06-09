#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitLocal(Node):
    """Map-frame pure pursuit that publishes the existing /cmd_vel interface."""

    def __init__(self):
        super().__init__('pure_pursuit_local')

        default_path = os.path.join(
            os.path.expanduser('~'),
            'f1',
            'src',
            'pure_pursuit',
            'paths',
            'path.csv',
        )

        self.declare_parameter('path_file', default_path)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('start_button_index', 1)
        self.declare_parameter('stop_button_index', 0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('min_lookahead', 0.45)
        self.declare_parameter('max_lookahead', 1.2)
        self.declare_parameter('use_dynamic_lookahead', False)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('steering_limit_deg', 22.0)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('linear_speed', 0.0)
        self.declare_parameter('publish_stop_when_disabled', True)
        self.declare_parameter('loop_path', True)
        self.declare_parameter('search_window', 80)

        self.path_file = str(self.get_parameter('path_file').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )
        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)
        self.use_dynamic_lookahead = bool(
            self.get_parameter('use_dynamic_lookahead').value
        )
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steering_limit = math.radians(
            float(self.get_parameter('steering_limit_deg').value)
        )
        self.steering_gain = float(self.get_parameter('steering_gain').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.publish_stop_when_disabled = bool(
            self.get_parameter('publish_stop_when_disabled').value
        )
        self.loop_path = bool(self.get_parameter('loop_path').value)
        self.search_window = int(self.get_parameter('search_window').value)
        self.drive_enabled = bool(self.get_parameter('auto_start').value)

        self.start_button_index = int(
            self.get_parameter('start_button_index').value
        )
        self.stop_button_index = int(
            self.get_parameter('stop_button_index').value
        )

        self.path = self.load_path(self.path_file)
        self.path_index = 0

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self.current_marker_pub = self.create_publisher(
            Marker,
            '/local_current_waypoint',
            10,
        )
        self.lookahead_marker_pub = self.create_publisher(
            Marker,
            '/local_lookahead_waypoint',
            10,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self.joy_callback,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        control_rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'Pure pursuit local loaded {len(self.path)} points from '
            f'{self.path_file}; publishing /cmd_vel compatible Twist'
        )
        if not self.drive_enabled:
            self.get_logger().info(
                f'Waiting for start button {self.start_button_index}; '
                f'stop button {self.stop_button_index}'
            )

    def load_path(self, path_file):
        points = []
        if not os.path.exists(path_file):
            self.get_logger().error(f'Path CSV does not exist: {path_file}')
            return points

        with open(path_file, newline='', encoding='utf-8') as csv_file:
            sample = csv_file.read(512)
            csv_file.seek(0)
            has_header = 'x' in sample.splitlines()[0].lower()
            reader = csv.DictReader(csv_file) if has_header else csv.reader(csv_file)
            for row in reader:
                try:
                    if has_header:
                        x = float(row['x'])
                        y = float(row['y'])
                        yaw = float(row.get('yaw', 0.0) or 0.0)
                    else:
                        x = float(row[0])
                        y = float(row[1])
                        yaw = float(row[2]) if len(row) > 2 else 0.0
                except (KeyError, TypeError, ValueError, IndexError):
                    continue
                points.append((x, y, yaw))

        if len(points) < 2:
            self.get_logger().error('Path CSV must contain at least 2 points')
        return points

    def joy_callback(self, msg):
        if self.button_pressed(msg, self.stop_button_index):
            if self.drive_enabled:
                self.get_logger().info('Stop button pressed')
            self.drive_enabled = False
            self.publish_stop()
            return

        if self.button_pressed(msg, self.start_button_index):
            if not self.drive_enabled:
                self.get_logger().info('Start button pressed')
            self.drive_enabled = True

    @staticmethod
    def button_pressed(msg, index):
        return index >= 0 and index < len(msg.buttons) and msg.buttons[index] == 1

    def control_loop(self):
        if len(self.path) < 2:
            self.publish_stop()
            return

        pose = self.lookup_pose()
        if pose is None:
            return
        x, y, yaw = pose

        if not self.drive_enabled:
            if self.publish_stop_when_disabled:
                self.publish_stop()
            return

        lookahead = self.compute_lookahead()
        nearest_index = self.find_nearest_index(x, y)
        target_index = self.find_target_index(x, y, nearest_index, lookahead)
        self.path_index = nearest_index

        target = self.path[target_index]
        steering = self.compute_steering(x, y, yaw, target)
        if self.invert_steering:
            steering *= -1.0
        steering = max(-self.steering_limit, min(self.steering_limit, steering))

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)

        self.publish_marker(self.current_marker_pub, self.path[nearest_index], 10, 0.0, 0.0, 1.0)
        self.publish_marker(self.lookahead_marker_pub, target, 11, 1.0, 0.0, 0.0)

    def lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.map_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def compute_lookahead(self):
        if not self.use_dynamic_lookahead:
            return self.lookahead_distance
        speed = abs(self.linear_speed)
        dynamic = max(self.min_lookahead, min(self.max_lookahead, speed * 0.8))
        return dynamic

    def find_nearest_index(self, x, y):
        if self.loop_path:
            candidate_indices = range(len(self.path))
        else:
            start = max(self.path_index - 5, 0)
            stop = min(self.path_index + self.search_window, len(self.path))
            candidate_indices = range(start, stop)

        best_index = self.path_index
        best_distance = float('inf')
        for index in candidate_indices:
            px, py, _ = self.path[index]
            distance = math.hypot(px - x, py - y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def find_target_index(self, x, y, nearest_index, lookahead):
        path_len = len(self.path)
        max_steps = path_len if self.loop_path else path_len - nearest_index
        best_index = nearest_index

        for step in range(max_steps):
            index = (nearest_index + step) % path_len
            px, py, _ = self.path[index]
            if math.hypot(px - x, py - y) >= lookahead:
                return index
            best_index = index
            if not self.loop_path and index == path_len - 1:
                break
        return best_index

    def compute_steering(self, x, y, yaw, target):
        target_x, target_y, _ = target
        dx = target_x - x
        dy = target_y - y
        target_angle = math.atan2(dy, dx)
        alpha = normalize_angle(target_angle - yaw)
        distance = max(math.hypot(dx, dy), 1e-3)
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        return steering * self.steering_gain

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_marker(self, publisher, point, marker_id, red, green, blue):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = marker_id
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.a = 1.0
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        publisher.publish(marker)


def main():
    rclpy.init()
    node = PurePursuitLocal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
