#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuBridge(Node):
    """Republish an existing IMU topic as /imu without touching the source node."""

    def __init__(self):
        super().__init__('imu_bridge')

        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('stamp_with_now', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.stamp_with_now = bool(self.get_parameter('stamp_with_now').value)

        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10,
        )

        self.get_logger().info(
            f'IMU bridge started: {input_topic} -> {output_topic}'
        )

    def imu_callback(self, msg):
        bridged = copy.deepcopy(msg)
        if self.frame_id:
            bridged.header.frame_id = self.frame_id
        if self.stamp_with_now:
            bridged.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(bridged)


def main():
    rclpy.init()
    node = ImuBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
