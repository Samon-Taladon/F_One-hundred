# #!/usr/bin/env python3

# import math
# from statistics import median
# from typing import List, Optional, Sequence, Tuple

# import rclpy
# from geometry_msgs.msg import Twist
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan


# class FollowGapNavigation(Node):
#     """LiDAR-only Follow The Gap controller that publishes geometry_msgs/Twist."""

#     def __init__(self):
#         super().__init__('follow_gap_navigation')

#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('scan_topic', '/scan'),
#                 ('cmd_vel_topic', '/cmd_vel'),
#                 ('control_rate', 20.0),
#                 ('scan_timeout', 0.50),
#                 ('max_speed', 1.5),
#                 ('min_speed', 0.5),
#                 ('normal_speed', 1.0),
#                 ('front_angle_range', 90.0),
#                 ('bubble_radius', 0.45),
#                 ('steering_gain', 1.0),
#                 ('max_steering', 0.6),
#                 ('smoothing_factor', 0.30),
#                 ('range_smoothing_window', 5),
#                 ('min_lidar_range', 0.05),
#                 ('max_lidar_range', 10.0),
#                 ('gap_clearance_threshold', 0.65),
#                 ('emergency_stop_distance', 0.35),
#                 ('slow_down_distance', 0.90),
#                 ('open_space_distance', 2.0),
#                 ('narrow_gap_width', 0.80),
#                 ('front_safety_angle', 12.0),
#                 ('best_point_depth_weight', 0.65),
#                 ('log_period', 1.0),
#             ],
#         )

#         scan_topic = self.get_parameter('scan_topic').value
#         cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

#         self.scan_sub = self.create_subscription(
#             LaserScan,
#             scan_topic,
#             self.scan_callback,
#             10,
#         )
#         self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

#         self.latest_scan: Optional[LaserScan] = None
#         self.latest_scan_time = self.get_clock().now()
#         self.filtered_steering = 0.0
#         self.last_log_time = self.get_clock().now()

#         control_rate = max(1.0, float(self.get_parameter('control_rate').value))
#         self.timer = self.create_timer(1.0 / control_rate, self.control_callback)

#         self.get_logger().info(
#             f'Follow Gap Navigation started: subscribe {scan_topic}, '
#             f'publish {cmd_vel_topic}'
#         )

#     def scan_callback(self, msg: LaserScan) -> None:
#         self.latest_scan = msg
#         self.latest_scan_time = self.get_clock().now()

#     def control_callback(self) -> None:
#         if self.latest_scan is None:
#             self.publish_stop('waiting for LiDAR scan')
#             return

#         scan_age = (
#             self.get_clock().now() - self.latest_scan_time
#         ).nanoseconds / 1e9
#         scan_timeout = float(self.get_parameter('scan_timeout').value)
#         if scan_age > scan_timeout:
#             self.publish_stop(f'LiDAR timeout {scan_age:.2f}s')
#             return

#         result = self.compute_command(self.latest_scan)
#         if result is None:
#             self.publish_stop('no safe gap found')
#             return

#         cmd, debug_text = result
#         self.cmd_pub.publish(cmd)
#         self.log_status(debug_text)

#     def compute_command(self, scan: LaserScan) -> Optional[Tuple[Twist, str]]:
#         angles, ranges = self.preprocess_scan(scan)
#         if not ranges:
#             return None

#         front_min = self.minimum_in_angle_window(
#             angles,
#             ranges,
#             math.radians(float(self.get_parameter('front_safety_angle').value)),
#         )
#         emergency_distance = float(
#             self.get_parameter('emergency_stop_distance').value
#         )
#         if front_min <= emergency_distance:
#             return self.emergency_stop_result(front_min)

#         closest_index = self.closest_obstacle_index(ranges)
#         if closest_index is None:
#             return None

#         candidate_ranges = list(ranges)
#         self.apply_safety_bubble(angles, candidate_ranges, closest_index)

#         gaps = self.find_gaps(candidate_ranges)
#         if not gaps:
#             return None

#         best_gap = max(gaps, key=lambda gap: self.gap_score(gap, candidate_ranges))
#         target_index = self.select_best_point(best_gap, angles, candidate_ranges)
#         if target_index is None:
#             return None

#         target_angle = angles[target_index]
#         target_range = candidate_ranges[target_index]
#         gap_width = self.estimate_gap_width(best_gap, angles, candidate_ranges)
#         steering = self.calculate_steering(target_angle)
#         speed = self.calculate_speed(front_min, target_range, gap_width)

#         cmd = Twist()
#         cmd.linear.x = speed
#         cmd.angular.z = steering

#         debug = (
#             f'target={math.degrees(target_angle):.1f}deg '
#             f'steering={steering:.3f} speed={speed:.2f}m/s '
#             f'front={front_min:.2f}m gap_width={gap_width:.2f}m'
#         )
#         return cmd, debug

#     def preprocess_scan(self, scan: LaserScan) -> Tuple[List[float], List[float]]:
#         front_angle = math.radians(
#             float(self.get_parameter('front_angle_range').value)
#         )
#         min_range = float(self.get_parameter('min_lidar_range').value)
#         max_range = float(self.get_parameter('max_lidar_range').value)

#         angles = []
#         ranges = []

#         for index, raw_range in enumerate(scan.ranges):
#             angle = scan.angle_min + index * scan.angle_increment
#             if angle < -front_angle or angle > front_angle:
#                 continue

#             # inf คือไกลมากจึง clamp เป็น max_range, nan/ค่าต่ำผิดปกติถือว่าใช้ไม่ได้
#             if math.isinf(raw_range):
#                 clean_range = max_range
#             elif math.isnan(raw_range) or raw_range < min_range:
#                 clean_range = 0.0
#             else:
#                 clean_range = min(raw_range, max_range)

#             angles.append(angle)
#             ranges.append(clean_range)

#         return angles, self.smooth_ranges(ranges)

#     def smooth_ranges(self, ranges: Sequence[float]) -> List[float]:
#         window = int(self.get_parameter('range_smoothing_window').value)
#         if window <= 1 or not ranges:
#             return list(ranges)

#         half_window = max(1, window // 2)
#         smoothed = []
#         for index in range(len(ranges)):
#             start = max(0, index - half_window)
#             stop = min(len(ranges), index + half_window + 1)
#             valid_values = [value for value in ranges[start:stop] if value > 0.0]
#             smoothed.append(median(valid_values) if valid_values else 0.0)
#         return smoothed

#     def closest_obstacle_index(self, ranges: Sequence[float]) -> Optional[int]:
#         valid = [
#             (index, value)
#             for index, value in enumerate(ranges)
#             if value > 0.0
#         ]
#         if not valid:
#             return None
#         return min(valid, key=lambda item: item[1])[0]

#     def apply_safety_bubble(
#         self,
#         angles: Sequence[float],
#         ranges: List[float],
#         obstacle_index: int,
#     ) -> None:
#         obstacle_distance = ranges[obstacle_index]
#         if obstacle_distance <= 0.0:
#             return

#         bubble_radius = float(self.get_parameter('bubble_radius').value)
#         bubble_angle = math.atan2(bubble_radius, obstacle_distance)
#         obstacle_angle = angles[obstacle_index]

#         # ลบ candidate รอบ obstacle ที่ใกล้ที่สุดออกจากการหา gap
#         for index, angle in enumerate(angles):
#             if abs(angle - obstacle_angle) <= bubble_angle:
#                 ranges[index] = 0.0

#     def find_gaps(self, ranges: Sequence[float]) -> List[Tuple[int, int]]:
#         threshold = float(self.get_parameter('gap_clearance_threshold').value)
#         gaps = []
#         start = None

#         for index, value in enumerate(ranges):
#             is_free = value >= threshold
#             if is_free and start is None:
#                 start = index
#             elif not is_free and start is not None:
#                 if index - start >= 2:
#                     gaps.append((start, index - 1))
#                 start = None

#         if start is not None and len(ranges) - start >= 2:
#             gaps.append((start, len(ranges) - 1))

#         return gaps

#     def gap_score(self, gap: Tuple[int, int], ranges: Sequence[float]) -> float:
#         start, stop = gap
#         width_points = stop - start + 1
#         depth = max(ranges[start:stop + 1])
#         return width_points * 0.8 + depth * 1.2

#     def select_best_point(
#         self,
#         gap: Tuple[int, int],
#         angles: Sequence[float],
#         ranges: Sequence[float],
#     ) -> Optional[int]:
#         start, stop = gap
#         if stop < start:
#             return None

#         edge_margin = max(1, int((stop - start + 1) * 0.12))
#         safe_start = min(stop, start + edge_margin)
#         safe_stop = max(start, stop - edge_margin)
#         if safe_stop < safe_start:
#             safe_start, safe_stop = start, stop

#         center_angle = (angles[start] + angles[stop]) * 0.5
#         max_range = max(ranges[safe_start:safe_stop + 1])
#         depth_weight = float(self.get_parameter('best_point_depth_weight').value)

#         best_index = safe_start
#         best_score = -1.0
#         for index in range(safe_start, safe_stop + 1):
#             depth_score = ranges[index] / max(max_range, 0.001)
#             center_score = 1.0 - min(
#                 1.0,
#                 abs(angles[index] - center_angle) / max(
#                     abs(angles[stop] - angles[start]) * 0.5,
#                     0.001,
#                 ),
#             )
#             score = depth_weight * depth_score + (1.0 - depth_weight) * center_score
#             if score > best_score:
#                 best_index = index
#                 best_score = score

#         return best_index

#     def estimate_gap_width(
#         self,
#         gap: Tuple[int, int],
#         angles: Sequence[float],
#         ranges: Sequence[float],
#     ) -> float:
#         start, stop = gap
#         left_range = ranges[start]
#         right_range = ranges[stop]
#         angle_width = abs(angles[stop] - angles[start])
#         average_range = max(0.0, (left_range + right_range) * 0.5)
#         return 2.0 * average_range * math.sin(angle_width * 0.5)

#     def calculate_steering(self, target_angle: float) -> float:
#         gain = float(self.get_parameter('steering_gain').value)
#         max_steering = float(self.get_parameter('max_steering').value)
#         smoothing = self.clamp(
#             float(self.get_parameter('smoothing_factor').value),
#             0.0,
#             1.0,
#         )

#         raw_steering = self.clamp(
#             gain * target_angle,
#             -max_steering,
#             max_steering,
#         )
#         self.filtered_steering = (
#             (1.0 - smoothing) * self.filtered_steering
#             + smoothing * raw_steering
#         )
#         return self.clamp(self.filtered_steering, -max_steering, max_steering)

#     def calculate_speed(
#         self,
#         front_min: float,
#         target_range: float,
#         gap_width: float,
#     ) -> float:
#         min_speed = float(self.get_parameter('min_speed').value)
#         normal_speed = float(self.get_parameter('normal_speed').value)
#         max_speed = float(self.get_parameter('max_speed').value)
#         slow_down_distance = float(self.get_parameter('slow_down_distance').value)
#         open_space_distance = float(self.get_parameter('open_space_distance').value)
#         narrow_gap_width = float(self.get_parameter('narrow_gap_width').value)

#         speed = normal_speed
#         if front_min < slow_down_distance or gap_width < narrow_gap_width:
#             speed = min_speed
#         elif target_range > open_space_distance and front_min > open_space_distance:
#             speed = max_speed

#         return self.clamp(speed, min_speed, max_speed)

#     def minimum_in_angle_window(
#         self,
#         angles: Sequence[float],
#         ranges: Sequence[float],
#         half_angle: float,
#     ) -> float:
#         values = [
#             value
#             for angle, value in zip(angles, ranges)
#             if abs(angle) <= half_angle and value > 0.0
#         ]
#         if not values:
#             return 0.0
#         return min(values)

#     def emergency_stop_result(self, front_min: float) -> Tuple[Twist, str]:
#         self.filtered_steering = 0.0
#         cmd = Twist()
#         debug = f'emergency stop: front obstacle {front_min:.2f}m'
#         return cmd, debug

#     def publish_stop(self, reason: str) -> None:
#         self.filtered_steering = 0.0
#         self.cmd_pub.publish(Twist())
#         self.log_status(f'stop: {reason}', warn=True)

#     def log_status(self, text: str, warn: bool = False) -> None:
#         now = self.get_clock().now()
#         period = float(self.get_parameter('log_period').value)
#         elapsed = (now - self.last_log_time).nanoseconds / 1e9
#         if elapsed < period:
#             return

#         if warn:
#             self.get_logger().warn(text)
#         else:
#             self.get_logger().info(text)
#         self.last_log_time = now

#     @staticmethod
#     def clamp(value: float, low: float, high: float) -> float:
#         return max(low, min(high, value))


# def main(args=None):
#     rclpy.init(args=args)
#     node = FollowGapNavigation()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.publish_stop('node shutdown')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import csv
import math
import os
from enum import Enum
from statistics import median
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class State(Enum):
    """Defines the operating state of the robot."""

    FOLLOW_GAP = 1
    EMERGENCY_STOP = 2
    REVERSING = 3


class FollowGapNavigation(Node):
    """
    LiDAR-only controller with Follow The Gap, Defensive Driving, and Stuck Recovery.
    """

    def __init__(self):
        super().__init__('follow_gap_navigation')

        # Declare all parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Topics and Timers
                ('scan_topic', '/scan'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('control_rate', 30.0),
                ('scan_timeout', 0.50),
                # Speed Control
                ('max_speed', 2.0),
                ('min_speed', 0.5),
                ('normal_speed', 1.5),
                # Adaptive Speed Profile
                ('adaptive_speed_enabled', False),
                ('adaptive_learning_enabled', False),
                ('waypoint_file', ''),
                ('speed_profile_file', ''),
                ('map_frame', 'map'),
                ('base_frame', 'base_link'),
                ('waypoint_search_window', 80),
                ('max_waypoint_distance', 0.75),
                ('learning_speed_increment', 0.05),
                ('learning_speed_penalty', 0.10),
                ('learning_safe_front_distance', 1.20),
                ('learning_safe_gap_width', 1.00),
                ('learning_max_steering', 0.35),
                ('profile_save_interval', 10),
                # LiDAR Processing
                ('front_angle_range', 120.0),
                ('rear_angle_range', 60.0),
                ('min_lidar_range', 0.05),
                ('max_lidar_range', 10.0),
                ('range_smoothing_window', 5),
                # Safety and Emergency
                ('emergency_stop_distance', 0.40),
                ('slow_down_distance', 0.90),
                ('front_safety_angle', 15.0),
                ('bubble_radius', 0.50),
                # Stuck Recovery (Reversing)
                ('reversing_enabled', True),
                ('stuck_timeout', 0.5),
                ('reversing_speed', 0.2),
                ('reverse_exit_distance', 0.65),
                # Gap Selection
                ('gap_clearance_threshold', 0.65),
                ('open_space_distance', 2.5),
                ('narrow_gap_width', 0.90),
                ('best_point_depth_weight', 0.6),
                # Steering Control
                ('steering_gain', 1.2),
                ('steering_direction', -1.0),
                ('max_steering', 0.65),
                ('smoothing_factor', 0.40),
                # Defensive Driving (Blocking)
                ('defensive_driving_enabled', False),
                ('opponent_behind_threshold', 1.5),
                ('blocking_gain', 0.3),
                # Logging
                ('log_period', 1.0),
            ],
        )

        # Get topic names from parameters
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Create subscriptions and publishers
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Initialize state variables
        self.state = State.FOLLOW_GAP
        self.state_enter_time = self.get_clock().now()
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = self.get_clock().now()
        self.filtered_steering = 0.0
        self.last_log_time = self.get_clock().now()

        # Adaptive speed profile state. The path heading is inferred from
        # waypoint order, so this feature does not depend on IMU yaw.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoints: List[Tuple[float, float]] = []
        self.learned_speeds: List[float] = []
        self.segment_visits: List[int] = []
        self.current_segment_index: Optional[int] = None
        self.segment_hazard = False
        self.segment_can_increase = True
        self.adaptive_pose_valid = False
        self.profile_updates_since_save = 0
        self.completed_laps = 0
        self.load_adaptive_speed_profile()

        # Start the main control loop
        control_rate = max(1.0, float(self.get_parameter('control_rate').value))
        self.timer = self.create_timer(1.0 / control_rate, self.control_callback)

        self.get_logger().info(
            f'Follow Gap Navigation started: subscribe {scan_topic}, '
            f'publish {cmd_vel_topic}'
        )

    def change_state(self, new_state: State):
        """Changes the robot's state and logs the transition."""
        if self.state != new_state:
            self.get_logger().info(f'State change: {self.state.name} -> {new_state.name}')
            self.state = new_state
            self.state_enter_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def control_callback(self) -> None:
        if self.latest_scan is None:
            self.publish_stop('waiting for LiDAR scan')
            return

        scan_age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
        scan_timeout = float(self.get_parameter('scan_timeout').value)
        if scan_age > scan_timeout:
            self.publish_stop(f'LiDAR timeout {scan_age:.2f}s')
            return

        result = self.compute_command(self.latest_scan)
        if result is None:
            self.publish_stop('no safe path found')
            return

        cmd, debug_text = result
        self.cmd_pub.publish(cmd)
        self.log_status(debug_text)

    def compute_command(self, scan: LaserScan) -> Optional[Tuple[Twist, str]]:
        # Preprocess scan to get front and rear data
        front_angles, front_ranges = self.preprocess_scan(scan, self.get_parameter('front_angle_range').value)
        opponent_side = self.find_opponent(scan)

        if not front_ranges:
            return self.create_emergency_stop_cmd('empty front scan')

        front_min = self.minimum_in_angle_window(
            front_angles,
            front_ranges,
            math.radians(float(self.get_parameter('front_safety_angle').value)),
        )

        # --- State Machine Logic ---
        now = self.get_clock().now()
        state_duration = (now - self.state_enter_time).nanoseconds / 1e9
        emergency_dist = self.get_parameter('emergency_stop_distance').value
        segment_index = self.update_adaptive_segment()

        # Transition conditions
        if self.state == State.FOLLOW_GAP and front_min <= emergency_dist:
            self.observe_adaptive_segment(hazard=True, can_increase=False)
            self.change_state(State.EMERGENCY_STOP)
            state_duration = 0.0
        elif self.state == State.EMERGENCY_STOP:
            stuck_timeout = self.get_parameter('stuck_timeout').value
            reverse_exit_distance = float(
                self.get_parameter('reverse_exit_distance').value
            )
            if front_min >= reverse_exit_distance:
                self.change_state(State.FOLLOW_GAP)
            elif (
                bool(self.get_parameter('reversing_enabled').value)
                and state_duration >= stuck_timeout
            ):
                self.change_state(State.REVERSING)
        elif self.state == State.REVERSING:
            reverse_exit_distance = float(
                self.get_parameter('reverse_exit_distance').value
            )
            if front_min >= reverse_exit_distance:
                self.change_state(State.FOLLOW_GAP)

        # Execute action based on current state
        if self.state == State.REVERSING:
            self.observe_adaptive_segment(hazard=True, can_increase=False)
            return self.create_reversing_cmd()

        if self.state == State.EMERGENCY_STOP:
            self.observe_adaptive_segment(hazard=True, can_increase=False)
            return self.create_emergency_stop_cmd(
                f'front obstacle {front_min:.2f}m'
            )

        # --- Default State: FOLLOW_GAP ---
        closest_index = self.closest_obstacle_index(front_ranges)
        if closest_index is None:
            self.observe_adaptive_segment(hazard=True, can_increase=False)
            return self.create_emergency_stop_cmd('no obstacles detected')

        candidate_ranges = list(front_ranges)
        self.apply_safety_bubble(front_angles, candidate_ranges, closest_index)

        gaps = self.find_gaps(candidate_ranges)
        if not gaps:
            self.observe_adaptive_segment(hazard=True, can_increase=False)
            return self.create_emergency_stop_cmd('no gaps found')

        best_gap = max(gaps, key=lambda gap: self.gap_score(gap, candidate_ranges))
        target_index = self.select_best_point(
            best_gap, front_angles, candidate_ranges
        )
        if target_index is None:
            self.observe_adaptive_segment(hazard=True, can_increase=False)
            return self.create_emergency_stop_cmd('could not select best point')

        target_angle = front_angles[target_index]
        target_range = candidate_ranges[target_index]
        gap_width = self.estimate_gap_width(best_gap, front_angles, candidate_ranges)

        # Defensive driving adjustment
        if self.get_parameter('defensive_driving_enabled').value and opponent_side:
            is_safe_to_block = front_min > self.get_parameter('slow_down_distance').value
            if is_safe_to_block:
                blocking_gain = self.get_parameter('blocking_gain').value
                if opponent_side == 'left':
                    target_angle += blocking_gain
                elif opponent_side == 'right':
                    target_angle -= blocking_gain
                self.get_logger().info(f"Blocking opponent on the {opponent_side}", throttle_duration_sec=1.0)


        steering = self.calculate_steering(target_angle)
        lidar_safe_speed = self.calculate_speed(
            front_min, target_range, gap_width
        )
        speed, profile_debug = self.apply_adaptive_speed(
            lidar_safe_speed, segment_index
        )

        learning_hazard = (
            front_min < float(
                self.get_parameter('learning_safe_front_distance').value
            )
            or gap_width < float(
                self.get_parameter('learning_safe_gap_width').value
            )
        )
        can_increase = (
            not learning_hazard
            and abs(steering) <= float(
                self.get_parameter('learning_max_steering').value
            )
            and lidar_safe_speed > speed + 1e-3
        )
        self.observe_adaptive_segment(
            hazard=learning_hazard,
            can_increase=can_increase,
        )

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering

        debug = (
            f'target={math.degrees(target_angle):.1f}deg steer={steering:.3f} speed={speed:.2f}m/s '
            f'front_min={front_min:.2f}m gap={gap_width:.2f}m'
            f'{profile_debug}'
        )
        return cmd, debug

    def load_adaptive_speed_profile(self) -> None:
        if not bool(self.get_parameter('adaptive_speed_enabled').value):
            return

        waypoint_file = os.path.expanduser(
            str(self.get_parameter('waypoint_file').value)
        )
        if not waypoint_file:
            self.get_logger().error(
                'adaptive_speed_enabled requires waypoint_file'
            )
            return

        try:
            with open(waypoint_file, newline='', encoding='utf-8') as csv_file:
                reader = csv.reader(csv_file)
                for row in reader:
                    if len(row) < 2:
                        continue
                    try:
                        self.waypoints.append((float(row[0]), float(row[1])))
                    except ValueError:
                        continue
        except OSError as exc:
            self.get_logger().error(
                f'Unable to load waypoint file {waypoint_file}: {exc}'
            )
            return

        if len(self.waypoints) < 2:
            self.get_logger().error(
                f'Waypoint file needs at least two points: {waypoint_file}'
            )
            self.waypoints = []
            return

        initial_speed = self.clamp(
            float(self.get_parameter('normal_speed').value),
            float(self.get_parameter('min_speed').value),
            float(self.get_parameter('max_speed').value),
        )
        self.learned_speeds = [initial_speed] * len(self.waypoints)
        self.segment_visits = [0] * len(self.waypoints)

        profile_file = self.profile_file_path()
        if profile_file and os.path.exists(profile_file):
            self.load_profile_file(profile_file)

        self.get_logger().info(
            f'Adaptive speed profile ready with {len(self.waypoints)} '
            f'waypoints; learning='
            f'{bool(self.get_parameter("adaptive_learning_enabled").value)}'
        )

    def load_profile_file(self, profile_file: str) -> None:
        try:
            with open(profile_file, newline='', encoding='utf-8') as csv_file:
                reader = csv.DictReader(csv_file)
                for row in reader:
                    index = int(row['index'])
                    if not 0 <= index < len(self.learned_speeds):
                        continue
                    waypoint_x, waypoint_y = self.waypoints[index]
                    if (
                        'x' in row
                        and 'y' in row
                        and math.hypot(
                            float(row['x']) - waypoint_x,
                            float(row['y']) - waypoint_y,
                        ) > 0.05
                    ):
                        continue
                    self.learned_speeds[index] = self.clamp(
                        float(row['speed']),
                        float(self.get_parameter('min_speed').value),
                        float(self.get_parameter('max_speed').value),
                    )
                    self.segment_visits[index] = int(row.get('visits', 0))
        except (OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'Ignoring invalid speed profile {profile_file}: {exc}'
            )

    def profile_file_path(self) -> str:
        configured_path = os.path.expanduser(
            str(self.get_parameter('speed_profile_file').value)
        )
        if configured_path:
            return configured_path

        waypoint_file = os.path.expanduser(
            str(self.get_parameter('waypoint_file').value)
        )
        if not waypoint_file:
            return ''
        root, _ = os.path.splitext(waypoint_file)
        return f'{root}_speed_profile.csv'

    def update_adaptive_segment(self) -> Optional[int]:
        self.adaptive_pose_valid = False
        if not self.waypoints:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter('map_frame').value),
                str(self.get_parameter('base_frame').value),
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Adaptive speed waiting for localization TF: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        position = transform.transform.translation
        segment_index, distance = self.find_nearest_waypoint(
            position.x, position.y
        )
        max_distance = float(
            self.get_parameter('max_waypoint_distance').value
        )
        if segment_index is None or distance > max_distance:
            self.get_logger().warn(
                f'Adaptive speed outside path by {distance:.2f}m',
                throttle_duration_sec=2.0,
            )
            return None

        self.adaptive_pose_valid = True
        if self.current_segment_index != segment_index:
            previous_index = self.current_segment_index
            if previous_index is not None:
                self.finalize_adaptive_segment(previous_index)
                if (
                    previous_index > len(self.waypoints) * 0.8
                    and segment_index < len(self.waypoints) * 0.2
                ):
                    self.completed_laps += 1
                    self.get_logger().info(
                        f'Adaptive speed completed lap {self.completed_laps}'
                    )
            self.current_segment_index = segment_index
            self.segment_hazard = False
            self.segment_can_increase = True

        return segment_index

    def find_nearest_waypoint(
        self, x_position: float, y_position: float
    ) -> Tuple[Optional[int], float]:
        if not self.waypoints:
            return None, float('inf')

        if self.current_segment_index is None:
            candidates = range(len(self.waypoints))
        else:
            window = max(
                1, int(self.get_parameter('waypoint_search_window').value)
            )
            candidates = (
                (self.current_segment_index + offset) % len(self.waypoints)
                for offset in range(window + 1)
            )

        best_index = None
        best_distance = float('inf')
        for index in candidates:
            waypoint_x, waypoint_y = self.waypoints[index]
            distance = math.hypot(
                x_position - waypoint_x, y_position - waypoint_y
            )
            if distance < best_distance:
                best_index = index
                best_distance = distance
        return best_index, best_distance

    def apply_adaptive_speed(
        self, lidar_safe_speed: float, segment_index: Optional[int]
    ) -> Tuple[float, str]:
        if segment_index is None or not self.learned_speeds:
            return lidar_safe_speed, ''

        profile_speed = self.learned_speeds[segment_index]
        speed = min(lidar_safe_speed, profile_speed)
        return (
            speed,
            f' profile={profile_speed:.2f}m/s segment={segment_index}',
        )

    def observe_adaptive_segment(
        self, hazard: bool, can_increase: bool
    ) -> None:
        if (
            self.current_segment_index is None
            or not self.adaptive_pose_valid
        ):
            return
        self.segment_hazard = self.segment_hazard or hazard
        self.segment_can_increase = (
            self.segment_can_increase and can_increase
        )

    def finalize_adaptive_segment(self, segment_index: int) -> None:
        if not bool(
            self.get_parameter('adaptive_learning_enabled').value
        ):
            return

        min_speed = float(self.get_parameter('min_speed').value)
        max_speed = float(self.get_parameter('max_speed').value)
        previous_speed = self.learned_speeds[segment_index]
        new_speed = previous_speed

        if self.segment_hazard:
            new_speed -= float(
                self.get_parameter('learning_speed_penalty').value
            )
        elif self.segment_can_increase:
            new_speed += float(
                self.get_parameter('learning_speed_increment').value
            )

        self.learned_speeds[segment_index] = self.clamp(
            new_speed, min_speed, max_speed
        )
        self.segment_visits[segment_index] += 1
        self.profile_updates_since_save += 1

        save_interval = max(
            1, int(self.get_parameter('profile_save_interval').value)
        )
        if self.profile_updates_since_save >= save_interval:
            self.save_adaptive_speed_profile()

    def save_adaptive_speed_profile(self) -> None:
        if not self.waypoints or not self.learned_speeds:
            return

        profile_file = self.profile_file_path()
        if not profile_file:
            return

        try:
            directory = os.path.dirname(os.path.abspath(profile_file))
            os.makedirs(directory, exist_ok=True)
            temporary_file = f'{profile_file}.tmp'
            with open(
                temporary_file, 'w', newline='', encoding='utf-8'
            ) as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(['index', 'x', 'y', 'speed', 'visits'])
                for index, waypoint in enumerate(self.waypoints):
                    writer.writerow([
                        index,
                        f'{waypoint[0]:.6f}',
                        f'{waypoint[1]:.6f}',
                        f'{self.learned_speeds[index]:.3f}',
                        self.segment_visits[index],
                    ])
            os.replace(temporary_file, profile_file)
            self.profile_updates_since_save = 0
        except OSError as exc:
            self.get_logger().error(
                f'Unable to save speed profile {profile_file}: {exc}'
            )

    def preprocess_scan(self, scan: LaserScan, angle_range_deg: float) -> Tuple[List[float], List[float]]:
        angle_range_rad = math.radians(angle_range_deg)
        min_range = float(self.get_parameter('min_lidar_range').value)
        max_range = float(self.get_parameter('max_lidar_range').value)

        angles, ranges = [], []
        for index, raw_range in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > angle_range_rad / 2.0:
                continue

            if math.isinf(raw_range):
                clean_range = max_range
            elif math.isnan(raw_range) or raw_range < min_range:
                clean_range = 0.0
            else:
                clean_range = min(raw_range, max_range)

            angles.append(angle)
            ranges.append(clean_range)

        return angles, self.smooth_ranges(ranges)

    def find_opponent(self, scan: LaserScan) -> Optional[str]:
        """Check for an opponent car behind."""
        rear_angle_deg = self.get_parameter('rear_angle_range').value
        min_range = self.get_parameter('min_lidar_range').value
        opponent_threshold = self.get_parameter('opponent_behind_threshold').value

        left_count = 0
        right_count = 0

        for index, raw_range in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            # Check rear-left and rear-right quadrants
            if math.pi - math.radians(rear_angle_deg) < angle < math.pi:
                 if min_range < raw_range < opponent_threshold:
                    right_count += 1
            elif -math.pi < angle < -math.pi + math.radians(rear_angle_deg):
                 if min_range < raw_range < opponent_threshold:
                    left_count += 1

        # Heuristic: if a cluster of points is detected, assume it's an opponent
        if left_count > 5:
            return 'left'
        if right_count > 5:
            return 'right'
        return None


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
        valid = [(i, r) for i, r in enumerate(ranges) if r > 0.0]
        return min(valid, key=lambda item: item[1])[0] if valid else None

    def apply_safety_bubble(
        self, angles: Sequence[float], ranges: List[float], obstacle_index: int
    ):
        obstacle_distance = ranges[obstacle_index]
        if obstacle_distance <= 0.0:
            return

        bubble_radius = float(self.get_parameter('bubble_radius').value)
        # Ensure argument for atan2 is valid
        if obstacle_distance > 0:
            bubble_angle = math.atan2(bubble_radius, obstacle_distance)
            obstacle_angle = angles[obstacle_index]
            for i, angle in enumerate(angles):
                if abs(angle - obstacle_angle) <= bubble_angle:
                    ranges[i] = 0.0

    def find_gaps(self, ranges: Sequence[float]) -> List[Tuple[int, int]]:
        threshold = float(self.get_parameter('gap_clearance_threshold').value)
        gaps, start = [], None
        for i, value in enumerate(ranges):
            is_free = value >= threshold
            if is_free and start is None:
                start = i
            elif not is_free and start is not None:
                if i - start >= 2:
                    gaps.append((start, i - 1))
                start = None
        if start is not None and len(ranges) - start >= 2:
            gaps.append((start, len(ranges) - 1))
        return gaps

    def gap_score(self, gap: Tuple[int, int], ranges: Sequence[float]) -> float:
        start, stop = gap
        width_points = stop - start + 1
        depth = max(ranges[start : stop + 1])
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

        # Avoid aiming at a gap edge, which makes the rear wheels cut corners.
        edge_margin = max(1, int((stop - start + 1) * 0.15))
        safe_start = min(stop, start + edge_margin)
        safe_stop = max(start, stop - edge_margin)
        if safe_stop < safe_start:
            safe_start, safe_stop = start, stop

        center_angle = (angles[start] + angles[stop]) * 0.5
        half_width = max(
            abs(angles[stop] - angles[start]) * 0.5,
            0.001,
        )
        max_range = max(ranges[safe_start : safe_stop + 1])
        depth_weight = self.clamp(
            float(self.get_parameter('best_point_depth_weight').value),
            0.0,
            1.0,
        )

        best_index = safe_start
        best_score = -1.0
        for index in range(safe_start, safe_stop + 1):
            depth_score = ranges[index] / max(max_range, 0.001)
            center_score = 1.0 - min(
                abs(angles[index] - center_angle) / half_width,
                1.0,
            )
            score = (
                depth_weight * depth_score
                + (1.0 - depth_weight) * center_score
            )
            if score > best_score:
                best_index = index
                best_score = score

        return best_index


    def estimate_gap_width(
        self, gap: Tuple[int, int], angles: Sequence[float], ranges: Sequence[float]
    ) -> float:
        start, stop = gap
        left_range = ranges[start]
        right_range = ranges[stop]
        angle_width = abs(angles[stop] - angles[start])
        return 2.0 * min(left_range, right_range) * math.sin(angle_width / 2.0)

    def calculate_steering(self, target_angle: float) -> float:
        gain = float(self.get_parameter('steering_gain').value)
        direction = (
            -1.0
            if float(self.get_parameter('steering_direction').value) >= 0.0
            else 1.0
        )
        max_steering = float(self.get_parameter('max_steering').value)
        smoothing = self.clamp(
            float(self.get_parameter('smoothing_factor').value), 0.0, 1.0
        )

        raw_steering = self.clamp(
            direction * gain * target_angle,
            -max_steering,
            max_steering,
        )
        self.filtered_steering = (
            (1.0 - smoothing) * self.filtered_steering
            + smoothing * raw_steering
        )
        return self.clamp(self.filtered_steering, -max_steering, max_steering)

    def calculate_speed(
        self, front_min: float, target_range: float, gap_width: float
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
        self, angles: Sequence[float], ranges: Sequence[float], half_angle: float
    ) -> float:
        values = [r for a, r in zip(angles, ranges) if abs(a) <= half_angle and r > 0.0]
        return min(values) if values else 0.0

    def create_reversing_cmd(self) -> Tuple[Twist, str]:
        """Creates a command to make the robot reverse."""
        reversing_speed = float(self.get_parameter('reversing_speed').value)
        cmd = Twist()
        cmd.linear.x = -reversing_speed
        cmd.angular.z = 0.0  # Or a slight turn if needed
        return cmd, f'reversing at {-reversing_speed:.2f}m/s'

    def create_emergency_stop_cmd(self, reason: str) -> Tuple[Twist, str]:
        """Creates a command for an emergency stop."""
        self.filtered_steering = 0.0
        return Twist(), f'emergency stop: {reason}'

    def publish_stop(self, reason: str) -> None:
        self.filtered_steering = 0.0
        self.cmd_pub.publish(Twist())
        self.log_status(f'stop: {reason}', warn=True)

    def log_status(self, text: str, warn: bool = False) -> None:
        now = self.get_clock().now()
        period = float(self.get_parameter('log_period').value)
        if (now - self.last_log_time).nanoseconds / 1e9 > period:
            (self.get_logger().warn if warn else self.get_logger().info)(text)
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
        node.save_adaptive_speed_profile()
        node.publish_stop('node shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
