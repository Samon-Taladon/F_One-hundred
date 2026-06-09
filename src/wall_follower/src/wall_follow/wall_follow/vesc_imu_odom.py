#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster


class VescImuOdom(Node):

    def __init__(self):
        super().__init__('vesc_imu_odom_node')

        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('motor_speed_topic', '/motor_speed')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_publish_rate', 20.0)
        self.declare_parameter('speed_timeout', 0.5)

        self.speed_to_erpm_gain = float(
            self.get_parameter('speed_to_erpm_gain').value
        )

        # Low pass filter coefficient
        self.alpha = 0.8

        # robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.current_angular_z = 0.0
        self.filtered_angular_z = 0.0
        self.current_speed = 0.0

        self.last_imu_time = None
        self.last_speed_msg_time = None
        self.last_publish_time = self.get_clock().now()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            10
        )
        self.speed_sub = self.create_subscription(
            Float64,
            self.get_parameter('motor_speed_topic').value,
            self.speed_callback,
            10
        )

        # publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10
        )
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('path_topic').value,
            10
        )

        # path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        publish_rate = float(self.get_parameter('odom_publish_rate').value)
        self.speed_timeout = float(self.get_parameter('speed_timeout').value)
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_timer_callback)

        self.get_logger().info("VESC IMU Odom + Path Node Started (with Low Pass Filter)")

    def imu_callback(self, msg):

        current_time = self.get_clock().now()

        if self.last_imu_time is not None:

            dt = (current_time - self.last_imu_time).nanoseconds / 1e9

            raw_angular_z = msg.angular_velocity.z

            # -------- Low Pass Filter --------
            self.filtered_angular_z = (
                self.alpha * self.filtered_angular_z +
                (1 - self.alpha) * raw_angular_z
            )

            # ตัด noise เล็ก ๆ
            if abs(self.filtered_angular_z) < 0.01:
                self.filtered_angular_z = 0.0

            self.current_angular_z = self.filtered_angular_z

            # integrate yaw
            self.yaw += self.current_angular_z * dt

        self.last_imu_time = current_time

    def speed_callback(self, msg):

        current_time = self.get_clock().now()

        speed = msg.data / self.speed_to_erpm_gain

        # ตัด noise
        if abs(speed) < 0.01:
            speed = 0.0

        self.current_speed = speed
        self.last_speed_msg_time = current_time

    def publish_timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = current_time

        speed = self.current_speed
        if self.last_speed_msg_time is None:
            speed = 0.0
        else:
            speed_age = (current_time - self.last_speed_msg_time).nanoseconds / 1e9
            if speed_age > self.speed_timeout:
                speed = 0.0
                self.current_speed = 0.0

        self.x += speed * math.cos(self.yaw) * dt
        self.y += speed * math.sin(self.yaw) * dt

        self.publish_all(current_time.to_msg(), speed)

    def publish_all(self, stamp, speed):

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # -------- TF --------
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # -------- ODOM --------
        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = self.current_angular_z

        self.odom_pub.publish(odom)

        # -------- PATH --------
        pose = PoseStamped()

        pose.header.stamp = stamp
        pose.header.frame_id = "odom"

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)

        self.path_pub.publish(self.path_msg)


def main():

    rclpy.init()

    node = VescImuOdom()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
