#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class LidarScanOdom(Node):
    def __init__(self):
        super().__init__('lidar_scan_odom')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('imu_yaw_source', 'orientation')
        self.declare_parameter('gyro_deadband_rad_s', 0.01)
        self.declare_parameter('min_range', 0.10)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('angle_min_deg', -180.0)
        self.declare_parameter('angle_max_deg', 180.0)
        self.declare_parameter('max_points', 360)
        self.declare_parameter('icp_iterations', 15)
        self.declare_parameter('max_correspondence_distance', 0.35)
        self.declare_parameter('min_correspondences', 35)
        self.declare_parameter('max_translation_per_scan', 0.50)
        self.declare_parameter('max_rotation_per_scan_deg', 25.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        self.imu_yaw_source = str(self.get_parameter('imu_yaw_source').value)
        self.gyro_deadband = float(self.get_parameter('gyro_deadband_rad_s').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10,
        )
        self.imu_sub = None
        if self.use_imu_yaw:
            self.imu_sub = self.create_subscription(
                Imu,
                self.get_parameter('imu_topic').value,
                self.imu_callback,
                25,
            )

        self.previous_points = None
        self.previous_stamp = None
        self.previous_imu_stamp = None
        self.imu_yaw = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info(
            f'lidar_scan_odom started: /scan -> {self.odom_topic} '
            f'using {"IMU yaw + LiDAR ICP translation" if self.use_imu_yaw else "LiDAR-only ICP"}'
        )

    def imu_callback(self, msg):
        if self.imu_yaw_source == 'gyro':
            stamp = msg.header.stamp
            if self.previous_imu_stamp is not None:
                dt = self.stamp_delta_seconds(self.previous_imu_stamp, stamp)
                angular_z = msg.angular_velocity.z
                if abs(angular_z) < self.gyro_deadband:
                    angular_z = 0.0
                self.imu_yaw = self.normalize_angle(
                    (self.imu_yaw or 0.0) + angular_z * dt
                )
            else:
                self.imu_yaw = 0.0
            self.previous_imu_stamp = stamp
            return

        self.imu_yaw = self.quaternion_to_yaw(msg.orientation)

    def scan_callback(self, scan):
        points = self.scan_to_points(scan)
        stamp = scan.header.stamp

        if points.shape[0] < self.get_parameter('min_correspondences').value:
            self.get_logger().warn(
                f'Not enough usable LiDAR points for odom: {points.shape[0]}'
            )
            return

        if self.previous_points is None:
            self.previous_points = points
            self.previous_stamp = stamp
            self.publish_odom(stamp, 0.0, 0.0, 0.0)
            self.get_logger().info('LiDAR odom origin initialized at x=0 y=0')
            return

        dx, dy, dyaw, ok = self.estimate_delta(points, self.previous_points)
        if ok:
            self.integrate_delta(dx, dy, dyaw)
            if self.use_imu_yaw and self.imu_yaw is not None:
                self.yaw = self.imu_yaw
            self.previous_points = points
        else:
            self.get_logger().warn('ICP update rejected; publishing last odom')
            if self.use_imu_yaw and self.imu_yaw is not None:
                self.yaw = self.imu_yaw

        dt = self.stamp_delta_seconds(self.previous_stamp, stamp)
        self.previous_stamp = stamp
        self.publish_odom(stamp, dx / dt if ok and dt > 0.0 else 0.0,
                          dy / dt if ok and dt > 0.0 else 0.0,
                          dyaw / dt if ok and dt > 0.0 else 0.0)

    def scan_to_points(self, scan):
        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)
        angle_min = math.radians(
            float(self.get_parameter('angle_min_deg').value)
        )
        angle_max = math.radians(
            float(self.get_parameter('angle_max_deg').value)
        )
        max_points = max(10, int(self.get_parameter('max_points').value))

        if angle_min > angle_max:
            angle_min, angle_max = angle_max, angle_min

        points = []
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if angle < angle_min or angle > angle_max:
                continue
            if not math.isfinite(value):
                continue
            if value < min_range or value > min(max_range, scan.range_max):
                continue
            points.append((value * math.cos(angle), value * math.sin(angle)))

        if len(points) > max_points:
            step = int(math.ceil(len(points) / max_points))
            points = points[::step]

        return np.asarray(points, dtype=np.float64)

    def estimate_delta(self, current_points, previous_points):
        iterations = int(self.get_parameter('icp_iterations').value)
        max_corr = float(
            self.get_parameter('max_correspondence_distance').value
        )
        min_pairs = int(self.get_parameter('min_correspondences').value)

        rotation = np.eye(2)
        translation = np.zeros(2)

        for _ in range(iterations):
            transformed = (rotation @ current_points.T).T + translation
            targets, mask = self.nearest_neighbors(
                transformed,
                previous_points,
                max_corr,
            )

            if int(mask.sum()) < min_pairs:
                return 0.0, 0.0, 0.0, False

            source = transformed[mask]
            target = targets[mask]
            delta_rotation, delta_translation = self.best_fit_transform(
                source,
                target,
            )

            rotation = delta_rotation @ rotation
            translation = delta_rotation @ translation + delta_translation

            if (
                np.linalg.norm(delta_translation) < 1e-4
                and abs(self.rotation_to_yaw(delta_rotation)) < 1e-4
            ):
                break

            max_corr *= 0.95

        dyaw = self.rotation_to_yaw(rotation)
        dx = float(translation[0])
        dy = float(translation[1])

        if not self.delta_is_reasonable(dx, dy, dyaw):
            return 0.0, 0.0, 0.0, False

        return dx, dy, dyaw, True

    def nearest_neighbors(self, source, target, max_corr):
        diff = source[:, None, :] - target[None, :, :]
        distances_sq = np.sum(diff * diff, axis=2)
        nearest_indices = np.argmin(distances_sq, axis=1)
        nearest_distances = np.sqrt(distances_sq[np.arange(source.shape[0]),
                                                 nearest_indices])
        return target[nearest_indices], nearest_distances <= max_corr

    def best_fit_transform(self, source, target):
        source_center = np.mean(source, axis=0)
        target_center = np.mean(target, axis=0)
        source_zero = source - source_center
        target_zero = target - target_center

        covariance = source_zero.T @ target_zero
        u_matrix, _, vt_matrix = np.linalg.svd(covariance)
        rotation = vt_matrix.T @ u_matrix.T

        if np.linalg.det(rotation) < 0.0:
            vt_matrix[-1, :] *= -1.0
            rotation = vt_matrix.T @ u_matrix.T

        translation = target_center - rotation @ source_center
        return rotation, translation

    def delta_is_reasonable(self, dx, dy, dyaw):
        max_translation = float(
            self.get_parameter('max_translation_per_scan').value
        )
        max_rotation = math.radians(
            float(self.get_parameter('max_rotation_per_scan_deg').value)
        )

        if math.hypot(dx, dy) > max_translation:
            return False
        if abs(dyaw) > max_rotation:
            return False
        return True

    def integrate_delta(self, dx, dy, dyaw):
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        self.x += (cos_yaw * dx) - (sin_yaw * dy)
        self.y += (sin_yaw * dx) + (cos_yaw * dy)
        if self.use_imu_yaw and self.imu_yaw is not None:
            self.yaw = self.imu_yaw
        else:
            self.yaw = self.normalize_angle(self.yaw + dyaw)

    def publish_odom(self, stamp, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.15
        odom.twist.covariance[0] = 0.10
        odom.twist.covariance[7] = 0.10
        odom.twist.covariance[35] = 0.20

        self.odom_pub.publish(odom)

        if self.publish_tf:
            self.publish_transform(stamp, odom.pose.pose.orientation)

    def publish_transform(self, stamp, orientation):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def yaw_to_quaternion(yaw):
        from geometry_msgs.msg import Quaternion

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    @staticmethod
    def rotation_to_yaw(rotation):
        return math.atan2(rotation[1, 0], rotation[0, 0])

    @staticmethod
    def quaternion_to_yaw(quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def stamp_delta_seconds(previous_stamp, current_stamp):
        if previous_stamp is None:
            return 0.0
        previous = previous_stamp.sec + previous_stamp.nanosec * 1e-9
        current = current_stamp.sec + current_stamp.nanosec * 1e-9
        return max(current - previous, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
