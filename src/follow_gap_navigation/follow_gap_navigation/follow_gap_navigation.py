#!/usr/bin/env python3

import math
from statistics import median
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FollowGapNavigation(Node):
    """LiDAR-only Follow The Gap controller that publishes geometry_msgs/Twist."""

    def __init__(self):
        super().__init__('follow_gap_navigation')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('scan_topic', '/scan'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('control_rate', 20.0),
                ('scan_timeout', 0.50),
                ('max_speed', 1.5),
                ('min_speed', 0.5),
                ('normal_speed', 1.0),
                ('front_angle_range', 90.0),
                ('bubble_radius', 0.45),
                ('steering_gain', 1.0),
                ('max_steering', 0.6),
                ('smoothing_factor', 0.30),
                ('range_smoothing_window', 5),
                ('min_lidar_range', 0.05),
                ('max_lidar_range', 10.0),
                ('gap_clearance_threshold', 0.65),
                ('emergency_stop_distance', 0.35),
                ('slow_down_distance', 0.90),
                ('open_space_distance', 2.0),
                ('narrow_gap_width', 0.80),
                ('front_safety_angle', 12.0),
                ('best_point_depth_weight', 0.65),
                ('log_period', 1.0),
            ],
        )

        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10,
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = self.get_clock().now()
        self.filtered_steering = 0.0
        self.last_log_time = self.get_clock().now()

        control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.timer = self.create_timer(1.0 / control_rate, self.control_callback)

        self.get_logger().info(
            f'Follow Gap Navigation started: subscribe {scan_topic}, '
            f'publish {cmd_vel_topic}'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def control_callback(self) -> None:
        if self.latest_scan is None:
            self.publish_stop('waiting for LiDAR scan')
            return

        scan_age = (
            self.get_clock().now() - self.latest_scan_time
        ).nanoseconds / 1e9
        scan_timeout = float(self.get_parameter('scan_timeout').value)
        if scan_age > scan_timeout:
            self.publish_stop(f'LiDAR timeout {scan_age:.2f}s')
            return

        result = self.compute_command(self.latest_scan)
        if result is None:
            self.publish_stop('no safe gap found')
            return

        cmd, debug_text = result
        self.cmd_pub.publish(cmd)
        self.log_status(debug_text)

    def compute_command(self, scan: LaserScan) -> Optional[Tuple[Twist, str]]:
        angles, ranges = self.preprocess_scan(scan)
        if not ranges:
            return None

        front_min = self.minimum_in_angle_window(
            angles,
            ranges,
            math.radians(float(self.get_parameter('front_safety_angle').value)),
        )
        emergency_distance = float(
            self.get_parameter('emergency_stop_distance').value
        )
        if front_min <= emergency_distance:
            return self.emergency_stop_result(front_min)

        closest_index = self.closest_obstacle_index(ranges)
        if closest_index is None:
            return None

        candidate_ranges = list(ranges)
        self.apply_safety_bubble(angles, candidate_ranges, closest_index)

        gaps = self.find_gaps(candidate_ranges)
        if not gaps:
            return None

        best_gap = max(gaps, key=lambda gap: self.gap_score(gap, candidate_ranges))
        target_index = self.select_best_point(best_gap, angles, candidate_ranges)
        if target_index is None:
            return None

        target_angle = angles[target_index]
        target_range = candidate_ranges[target_index]
        gap_width = self.estimate_gap_width(best_gap, angles, candidate_ranges)
        steering = self.calculate_steering(target_angle)
        speed = self.calculate_speed(front_min, target_range, gap_width)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering

        debug = (
            f'target={math.degrees(target_angle):.1f}deg '
            f'steering={steering:.3f} speed={speed:.2f}m/s '
            f'front={front_min:.2f}m gap_width={gap_width:.2f}m'
        )
        return cmd, debug

    def preprocess_scan(self, scan: LaserScan) -> Tuple[List[float], List[float]]:
        front_angle = math.radians(
            float(self.get_parameter('front_angle_range').value)
        )
        min_range = float(self.get_parameter('min_lidar_range').value)
        max_range = float(self.get_parameter('max_lidar_range').value)

        angles = []
        ranges = []

        for index, raw_range in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if angle < -front_angle or angle > front_angle:
                continue

            # inf คือไกลมากจึง clamp เป็น max_range, nan/ค่าต่ำผิดปกติถือว่าใช้ไม่ได้
            if math.isinf(raw_range):
                clean_range = max_range
            elif math.isnan(raw_range) or raw_range < min_range:
                clean_range = 0.0
            else:
                clean_range = min(raw_range, max_range)

            angles.append(angle)
            ranges.append(clean_range)

        return angles, self.smooth_ranges(ranges)

    def smooth_ranges(self, ranges: Sequence[float]) -> List[float]:
        window = int(self.get_parameter('range_smoothing_window').value)
        if window <= 1 or not ranges:
            return list(ranges)

        half_window = max(1, window // 2)
        smoothed = []
        for index in range(len(ranges)):
            start = max(0, index - half_window)
            stop = min(len(ranges), index + half_window + 1)
            valid_values = [value for value in ranges[start:stop] if value > 0.0]
            smoothed.append(median(valid_values) if valid_values else 0.0)
        return smoothed

    def closest_obstacle_index(self, ranges: Sequence[float]) -> Optional[int]:
        valid = [
            (index, value)
            for index, value in enumerate(ranges)
            if value > 0.0
        ]
        if not valid:
            return None
        return min(valid, key=lambda item: item[1])[0]

    def apply_safety_bubble(
        self,
        angles: Sequence[float],
        ranges: List[float],
        obstacle_index: int,
    ) -> None:
        obstacle_distance = ranges[obstacle_index]
        if obstacle_distance <= 0.0:
            return

        bubble_radius = float(self.get_parameter('bubble_radius').value)
        bubble_angle = math.atan2(bubble_radius, obstacle_distance)
        obstacle_angle = angles[obstacle_index]

        # ลบ candidate รอบ obstacle ที่ใกล้ที่สุดออกจากการหา gap
        for index, angle in enumerate(angles):
            if abs(angle - obstacle_angle) <= bubble_angle:
                ranges[index] = 0.0

    def find_gaps(self, ranges: Sequence[float]) -> List[Tuple[int, int]]:
        threshold = float(self.get_parameter('gap_clearance_threshold').value)
        gaps = []
        start = None

        for index, value in enumerate(ranges):
            is_free = value >= threshold
            if is_free and start is None:
                start = index
            elif not is_free and start is not None:
                if index - start >= 2:
                    gaps.append((start, index - 1))
                start = None

        if start is not None and len(ranges) - start >= 2:
            gaps.append((start, len(ranges) - 1))

        return gaps

    def gap_score(self, gap: Tuple[int, int], ranges: Sequence[float]) -> float:
        start, stop = gap
        width_points = stop - start + 1
        depth = max(ranges[start:stop + 1])
        return width_points * 0.8 + depth * 1.2

    def select_best_point(
        self,
        gap: Tuple[int, int],
        angles: Sequence[float],
        ranges: Sequence[float],
    ) -> Optional[int]:
        start, stop = gap
        if stop < start:
            return None

        edge_margin = max(1, int((stop - start + 1) * 0.12))
        safe_start = min(stop, start + edge_margin)
        safe_stop = max(start, stop - edge_margin)
        if safe_stop < safe_start:
            safe_start, safe_stop = start, stop

        center_angle = (angles[start] + angles[stop]) * 0.5
        max_range = max(ranges[safe_start:safe_stop + 1])
        depth_weight = float(self.get_parameter('best_point_depth_weight').value)

        best_index = safe_start
        best_score = -1.0
        for index in range(safe_start, safe_stop + 1):
            depth_score = ranges[index] / max(max_range, 0.001)
            center_score = 1.0 - min(
                1.0,
                abs(angles[index] - center_angle) / max(
                    abs(angles[stop] - angles[start]) * 0.5,
                    0.001,
                ),
            )
            score = depth_weight * depth_score + (1.0 - depth_weight) * center_score
            if score > best_score:
                best_index = index
                best_score = score

        return best_index

    def estimate_gap_width(
        self,
        gap: Tuple[int, int],
        angles: Sequence[float],
        ranges: Sequence[float],
    ) -> float:
        start, stop = gap
        left_range = ranges[start]
        right_range = ranges[stop]
        angle_width = abs(angles[stop] - angles[start])
        average_range = max(0.0, (left_range + right_range) * 0.5)
        return 2.0 * average_range * math.sin(angle_width * 0.5)

    def calculate_steering(self, target_angle: float) -> float:
        gain = float(self.get_parameter('steering_gain').value)
        max_steering = float(self.get_parameter('max_steering').value)
        smoothing = self.clamp(
            float(self.get_parameter('smoothing_factor').value),
            0.0,
            1.0,
        )

        raw_steering = self.clamp(
            gain * target_angle,
            -max_steering,
            max_steering,
        )
        self.filtered_steering = (
            (1.0 - smoothing) * self.filtered_steering
            + smoothing * raw_steering
        )
        return self.clamp(self.filtered_steering, -max_steering, max_steering)

    def calculate_speed(
        self,
        front_min: float,
        target_range: float,
        gap_width: float,
    ) -> float:
        min_speed = float(self.get_parameter('min_speed').value)
        normal_speed = float(self.get_parameter('normal_speed').value)
        max_speed = float(self.get_parameter('max_speed').value)
        slow_down_distance = float(self.get_parameter('slow_down_distance').value)
        open_space_distance = float(self.get_parameter('open_space_distance').value)
        narrow_gap_width = float(self.get_parameter('narrow_gap_width').value)

        speed = normal_speed
        if front_min < slow_down_distance or gap_width < narrow_gap_width:
            speed = min_speed
        elif target_range > open_space_distance and front_min > open_space_distance:
            speed = max_speed

        return self.clamp(speed, min_speed, max_speed)

    def minimum_in_angle_window(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
        half_angle: float,
    ) -> float:
        values = [
            value
            for angle, value in zip(angles, ranges)
            if abs(angle) <= half_angle and value > 0.0
        ]
        if not values:
            return 0.0
        return min(values)

    def emergency_stop_result(self, front_min: float) -> Tuple[Twist, str]:
        self.filtered_steering = 0.0
        cmd = Twist()
        debug = f'emergency stop: front obstacle {front_min:.2f}m'
        return cmd, debug

    def publish_stop(self, reason: str) -> None:
        self.filtered_steering = 0.0
        self.cmd_pub.publish(Twist())
        self.log_status(f'stop: {reason}', warn=True)

    def log_status(self, text: str, warn: bool = False) -> None:
        now = self.get_clock().now()
        period = float(self.get_parameter('log_period').value)
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed < period:
            return

        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self.last_log_time = now

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = FollowGapNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop('node shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
