#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_from_transform(transform):
    return yaw_from_quaternion(transform.rotation)


class TrajectoryLogger(Node):
    """Record x,y,yaw from localization into a CSV path."""

    def __init__(self):
        super().__init__('trajectory_logger')

        default_output = os.path.join(
            os.path.expanduser('~'),
            'f1',
            'src',
            'pure_pursuit',
            'paths',
            f'path_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
        )

        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('pose_topic', '/localization_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_path', default_output)
        self.declare_parameter('min_distance', 0.2)
        self.declare_parameter('timer_rate', 20.0)
        self.declare_parameter('append', False)
        self.declare_parameter('auto_start', True)

        self.pose_source = str(self.get_parameter('pose_source').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.output_path = str(self.get_parameter('output_path').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.recording = bool(self.get_parameter('auto_start').value)

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        file_exists = os.path.exists(self.output_path)
        mode = 'a' if bool(self.get_parameter('append').value) else 'w'
        self.csv_file = open(self.output_path, mode, newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        if mode == 'w' or not file_exists:
            self.writer.writerow(['x', 'y', 'yaw'])
            self.csv_file.flush()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_x = None
        self.last_y = None
        self.saved_count = 0

        if self.pose_source == 'pose':
            self.pose_sub = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                10,
            )
        elif self.pose_source == 'odom':
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                10,
            )
        else:
            rate = max(float(self.get_parameter('timer_rate').value), 1.0)
            self.timer = self.create_timer(1.0 / rate, self.tf_timer_callback)

        self.get_logger().info(
            f'Trajectory logger writing {self.output_path}; '
            f'source={self.pose_source}, min_distance={self.min_distance:.3f} m'
        )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()

    def pose_callback(self, msg):
        yaw = yaw_from_quaternion(msg.pose.orientation)
        self.maybe_save(msg.pose.position.x, msg.pose.position.y, yaw)

    def odom_callback(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.maybe_save(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def tf_timer_callback(self):
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
            return

        translation = transform.transform.translation
        yaw = yaw_from_transform(transform.transform)
        self.maybe_save(translation.x, translation.y, yaw)

    def maybe_save(self, x, y, yaw):
        if not self.recording:
            return

        if self.last_x is not None:
            distance = math.hypot(x - self.last_x, y - self.last_y)
            if distance < self.min_distance:
                return

        self.writer.writerow([f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
        self.csv_file.flush()
        self.last_x = x
        self.last_y = y
        self.saved_count += 1
        self.get_logger().info(
            f'Saved path point {self.saved_count}: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )


def main():
    rclpy.init()
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
