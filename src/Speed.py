import rclpy
from rclpy.node import Node

from vesc_msgs.msg import VescStateStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import math


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_node')

        # parameters (ต้องใส่ค่าตามรถจริง)
        self.pole_pairs = 7
        self.gear_ratio = 8.4
        self.wheel_diameter = 0.1   # meter

        self.erpm = 0.0
        self.omega = 0.0

        # subscribers
        self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.vesc_callback,
            10)

        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)

        # publishers
        self.v_pub = self.create_publisher(Float64, '/linear_velocity', 10)
        self.w_pub = self.create_publisher(Float64, '/angular_velocity', 10)

        # timer
        self.timer = self.create_timer(0.02, self.publish_velocity)  # 50 Hz

    def vesc_callback(self, msg):
        self.erpm = msg.state.speed

    def imu_callback(self, msg):
        self.omega = msg.angular_velocity.z

    def calculate_speed(self):

        v = (self.erpm / (self.pole_pairs * self.gear_ratio)) * math.pi * self.wheel_diameter

        return v

    def publish_velocity(self):

        v = self.calculate_speed()
        w = self.omega

        v_msg = Float64()
        v_msg.data = v

        w_msg = Float64()
        w_msg.data = w

        self.v_pub.publish(v_msg)
        self.w_pub.publish(w_msg)

        self.get_logger().info(f"v = {v:.3f} m/s , w = {w:.3f} rad/s")


def main(args=None):

    rclpy.init(args=args)

    node = VelocityPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()