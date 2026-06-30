#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from vesc_msgs.msg import VescStateStamped


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VescToOdom(Node):
    def __init__(self):
        super().__init__('vesc_to_odom_node')

        # ---- topics / frames ----
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('path_topic', '/path_raw')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # IMPORTANT: confirm this matches your real VESC feedback topic.
        # Your driver currently only shows /commands/motor/speed (a command, not feedback).
        # If your driver doesn't publish VescStateStamped on /sensors/core,
        # this node will never receive speed data -- check `ros2 node info /vesc_driver_node`.
        self.declare_parameter('vesc_state_topic', '/sensors/core')
        self.declare_parameter('servo_topic', '/commands/servo/position')

        # ---- VESC speed conversion ----
        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        self.declare_parameter('speed_deadband', 0.01)
        self.declare_parameter('speed_timeout', 0.5)

        # ---- Ackermann steering conversion ----
        self.declare_parameter('wheelbase', 0.33)
        # steering_angle = (servo_cmd - steering_to_servo_offset) / steering_to_servo_gain
        self.declare_parameter('steering_to_servo_gain', -0.5)
        self.declare_parameter('steering_to_servo_offset', 0.5)
        self.declare_parameter('servo_timeout', 0.5)

        # ---- misc ----
        self.declare_parameter('publish_tf', False)  # EKF owns odom->base_link TF
        self.declare_parameter('odom_publish_rate', 30.0)
        self.declare_parameter('max_path_length', 5000)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.speed_to_erpm_gain = float(self.get_parameter('speed_to_erpm_gain').value)
        self.speed_deadband = float(self.get_parameter('speed_deadband').value)
        self.speed_timeout = float(self.get_parameter('speed_timeout').value)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steering_gain = float(self.get_parameter('steering_to_servo_gain').value)
        self.steering_offset = float(self.get_parameter('steering_to_servo_offset').value)
        self.servo_timeout = float(self.get_parameter('servo_timeout').value)

        self.max_path_length = int(self.get_parameter('max_path_length').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.current_speed = 0.0
        self.steering_angle = 0.0

        self.last_speed_msg_time = None
        self.last_servo_msg_time = None
        self.last_publish_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)

        vesc_topic = self.get_parameter('vesc_state_topic').value
        servo_topic = self.get_parameter('servo_topic').value

        self.create_subscription(
            VescStateStamped,
            vesc_topic,
            self.speed_callback,
            10
        )

        self.create_subscription(
            Float64,
            servo_topic,
            self.servo_callback,
            10
        )

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

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame

        publish_rate = float(self.get_parameter('odom_publish_rate').value)
        self.create_timer(1.0 / publish_rate, self.publish_timer_callback)

        self.get_logger().info(
            f'VESC-to-odom node started. Subscribing to vesc="{vesc_topic}" servo="{servo_topic}". '
            f'Publishing odom on "{self.get_parameter("odom_topic").value}".'
        )

    def speed_callback(self, msg):
        current_time = self.get_clock().now()

        erpm = msg.state.speed
        speed = erpm / self.speed_to_erpm_gain

        if abs(speed) < self.speed_deadband:
            speed = 0.0

        self.current_speed = speed
        self.last_speed_msg_time = current_time

    def servo_callback(self, msg):
        current_time = self.get_clock().now()

        if abs(self.steering_gain) > 1e-9:
            self.steering_angle = (msg.data - self.steering_offset) / self.steering_gain
        else:
            self.steering_angle = 0.0

        self.last_servo_msg_time = current_time

    def publish_timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        speed = self.current_speed
        if self.last_speed_msg_time is None:
            speed = 0.0
        else:
            speed_age = (now - self.last_speed_msg_time).nanoseconds / 1e9
            if speed_age > self.speed_timeout:
                speed = 0.0
                self.current_speed = 0.0

        steering_angle = self.steering_angle
        if self.last_servo_msg_time is None:
            steering_angle = 0.0
        else:
            servo_age = (now - self.last_servo_msg_time).nanoseconds / 1e9
            if servo_age > self.servo_timeout:
                steering_angle = 0.0
                self.steering_angle = 0.0

        # Ackermann bicycle model yaw rate -- instant response, no integration lag
        if abs(self.wheelbase) > 1e-6:
            angular_z = (speed / self.wheelbase) * math.tan(steering_angle)
        else:
            angular_z = 0.0

        self.yaw = wrap_angle(self.yaw + angular_z * dt)

        self.x += speed * math.cos(self.yaw) * dt
        self.y += speed * math.sin(self.yaw) * dt

        self.publish_all(now.to_msg(), speed, angular_z)

    def publish_all(self, stamp, speed, angular_z):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = angular_z

        # Wheel-odom-only estimate: trust speed & yaw rate from kinematics,
        # but DO NOT claim high confidence in absolute yaw -- let the IMU
        # correct heading inside the EKF. Hence higher yaw covariance here.
        odom.pose.covariance[0] = 0.05    # x
        odom.pose.covariance[7] = 0.05    # y
        odom.pose.covariance[35] = 0.20   # yaw -- looser, IMU will refine this in EKF

        odom.twist.covariance[0] = 0.05   # linear x velocity -- fairly trusted
        odom.twist.covariance[35] = 0.10  # angular z velocity

        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.odom_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > self.max_path_length:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]

        self.path_pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = VescToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()