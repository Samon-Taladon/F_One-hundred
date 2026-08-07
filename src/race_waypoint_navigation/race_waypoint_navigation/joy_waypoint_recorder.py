#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


class JoyWaypointRecorder(Node):
    """Record map-frame waypoints while a joystick button is held or toggled."""

    def __init__(self):
        super().__init__('joy_waypoint_recorder')

        default_output = os.path.join(
            os.path.expanduser('~'),
            'f1',
            'src',
            'race_waypoint_navigation',
            'waypoints',
            f'raw_path_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
        )

        self.declare_parameter('output_path', default_output)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('record_button_index', 4)
        self.declare_parameter('stop_button_index', 0)
        self.declare_parameter('toggle_mode', False)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('min_distance', 0.15)
        self.declare_parameter('timer_rate', 30.0)

        self.output_path = os.path.expanduser(
            str(self.get_parameter('output_path').value)
        )
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.record_button_index = int(
            self.get_parameter('record_button_index').value
        )
        self.stop_button_index = int(self.get_parameter('stop_button_index').value)
        self.toggle_mode = bool(self.get_parameter('toggle_mode').value)
        self.recording = bool(self.get_parameter('auto_start').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.last_buttons = []
        self.last_x = None
        self.last_y = None
        self.saved_count = 0

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.csv_file = open(self.output_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['x', 'y', 'yaw'])
        self.csv_file.flush()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self.joy_callback,
            10,
        )

        timer_rate = max(float(self.get_parameter('timer_rate').value), 1.0)
        self.timer = self.create_timer(1.0 / timer_rate, self.timer_callback)

        mode = 'toggle' if self.toggle_mode else 'hold'
        self.get_logger().info(
            f'Recording waypoints to {self.output_path}; mode={mode}, '
            f'record_button={self.record_button_index}, '
            f'stop_button={self.stop_button_index}, auto_start={self.recording}'
        )

    def joy_callback(self, msg):
        if self.button_pressed(msg, self.stop_button_index):
            self.recording = False
            self.get_logger().info('Recording stopped by joystick stop button')
            return

        if self.toggle_mode:
            if self.rising_edge(msg, self.record_button_index):
                self.recording = not self.recording
                state = 'started' if self.recording else 'paused'
                self.get_logger().info(f'Recording {state}')
        else:
            self.recording = self.button_pressed(msg, self.record_button_index)

        self.last_buttons = list(msg.buttons)

    def timer_callback(self):
        if not self.recording:
            return

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
        yaw = yaw_from_quaternion(transform.transform.rotation)
        self.maybe_save(translation.x, translation.y, yaw)

    def maybe_save(self, x, y, yaw):
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
            f'Saved waypoint {self.saved_count}: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()

    @staticmethod
    def button_pressed(msg, index):
        return index >= 0 and index < len(msg.buttons) and msg.buttons[index] == 1

    def rising_edge(self, msg, index):
        if index < 0 or index >= len(msg.buttons):
            return False
        previous = index < len(self.last_buttons) and self.last_buttons[index] == 1
        return msg.buttons[index] == 1 and not previous


def main(args=None):
    rclpy.init(args=args)
    node = JoyWaypointRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
