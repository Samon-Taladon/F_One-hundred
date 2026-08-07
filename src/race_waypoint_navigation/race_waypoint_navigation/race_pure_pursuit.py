#!/usr/bin/env python3
import csv
import math
import os
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class RaceState(Enum):
    FOLLOW_WAYPOINT = 'FOLLOW_WAYPOINT'
    SLOW_DOWN = 'SLOW_DOWN'
    AVOID_GAP = 'AVOID_GAP'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_yaw, cos_yaw)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class RacePurePursuit(Node):
    """Pure pursuit race controller with a LiDAR obstacle safety layer."""

    def __init__(self):
        super().__init__('race_pure_pursuit')

        default_path = os.path.join(
            os.path.expanduser('~'),
            'f1',
            'src',
            'race_waypoint_navigation',
            'waypoints',
            'raceline.csv',
        )

        self.declare_parameter('path_file', default_path)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('start_button_index', 1)
        self.declare_parameter('stop_button_index', 0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('loop_path', True)
        self.declare_parameter('search_window', 120)
        self.declare_parameter('min_lap_time', 5.0)
        self.declare_parameter('default_speed', 0.35)
        self.declare_parameter('max_speed', 1.2)
        self.declare_parameter('min_moving_speed', 0.18)
        self.declare_parameter('default_lookahead', 0.75)
        self.declare_parameter('min_lookahead', 0.4)
        self.declare_parameter('max_lookahead', 1.4)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('steering_limit_deg', 22.0)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('front_angle_range_deg', 70.0)
        self.declare_parameter('emergency_stop_distance', 0.35)
        self.declare_parameter('slow_down_distance', 1.0)
        self.declare_parameter('avoidance_enter_distance', 0.7)
        self.declare_parameter('avoidance_exit_distance', 1.1)
        self.declare_parameter('avoidance_speed', 0.25)
        self.declare_parameter('gap_min_distance', 0.45)
        self.declare_parameter('gap_bubble_radius', 0.35)
        self.declare_parameter('gap_steering_gain', 1.0)
        self.declare_parameter('corridor_width', 1.2)
        self.declare_parameter('return_error_threshold', 0.35)
        self.declare_parameter('publish_markers', True)

        self.path_file = os.path.expanduser(str(self.get_parameter('path_file').value))
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)
        self.search_window = int(self.get_parameter('search_window').value)
        self.min_lap_time = float(self.get_parameter('min_lap_time').value)
        self.default_speed = float(self.get_parameter('default_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_moving_speed = float(self.get_parameter('min_moving_speed').value)
        self.default_lookahead = float(self.get_parameter('default_lookahead').value)
        self.min_lookahead = float(self.get_parameter('min_lookahead').value)
        self.max_lookahead = float(self.get_parameter('max_lookahead').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steering_limit = math.radians(
            float(self.get_parameter('steering_limit_deg').value)
        )
        self.steering_gain = float(self.get_parameter('steering_gain').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)

        self.path = self.load_path(self.path_file)
        self.path_index = 0
        self.drive_enabled = bool(self.get_parameter('auto_start').value)
        self.state = RaceState.FOLLOW_WAYPOINT
        self.latest_scan = None
        self.latest_scan_time = self.get_clock().now()
        self.lap_count = 0
        self.last_lap_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            10,
        )
        self.state_pub = self.create_publisher(String, '/race_state', 10)
        self.target_marker_pub = self.create_publisher(
            Marker,
            '/race_target_waypoint',
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.scan_callback,
            10,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            str(self.get_parameter('joy_topic').value),
            self.joy_callback,
            10,
        )

        control_rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'Race pure pursuit loaded {len(self.path)} points from '
            f'{self.path_file}; auto_start={self.drive_enabled}'
        )

    def load_path(self, path_file):
        rows = []
        if not os.path.exists(path_file):
            self.get_logger().error(f'Raceline CSV does not exist: {path_file}')
            return rows

        with open(path_file, newline='', encoding='utf-8') as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames and 'x' in reader.fieldnames:
                for row in reader:
                    try:
                        rows.append({
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'yaw': float(row.get('yaw', 0.0) or 0.0),
                            'target_speed': float(
                                row.get('target_speed', self.default_speed) or self.default_speed
                            ),
                            'lookahead': float(
                                row.get('lookahead', self.default_lookahead)
                                or self.default_lookahead
                            ),
                        })
                    except (KeyError, TypeError, ValueError):
                        continue
                return rows

        with open(path_file, newline='', encoding='utf-8') as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                try:
                    rows.append({
                        'x': float(row[0]),
                        'y': float(row[1]),
                        'yaw': float(row[2]) if len(row) > 2 else 0.0,
                        'target_speed': self.default_speed,
                        'lookahead': self.default_lookahead,
                    })
                except (TypeError, ValueError, IndexError):
                    continue
        return rows

    def joy_callback(self, msg):
        stop_index = int(self.get_parameter('stop_button_index').value)
        start_index = int(self.get_parameter('start_button_index').value)
        if self.button_pressed(msg, stop_index):
            self.drive_enabled = False
            self.publish_stop()
            self.get_logger().info('Race controller stopped by joystick')
            return
        if self.button_pressed(msg, start_index):
            if not self.drive_enabled:
                self.get_logger().info('Race controller started by joystick')
            self.drive_enabled = True

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def control_loop(self):
        if len(self.path) < 2:
            self.publish_stop()
            return

        pose = self.lookup_pose()
        if pose is None:
            self.publish_stop()
            return
        x, y, yaw = pose

        if not self.drive_enabled:
            self.publish_stop()
            self.publish_state()
            return

        nearest_index, cross_track_error = self.find_nearest_index(x, y)
        self.update_lap_count(nearest_index)
        self.path_index = nearest_index

        obstacle = self.analyze_scan()
        self.update_state(obstacle, cross_track_error)

        if self.state == RaceState.EMERGENCY_STOP:
            self.publish_stop()
            self.publish_state()
            return

        if self.state == RaceState.AVOID_GAP and obstacle['gap_angle'] is not None:
            cmd = self.make_gap_command(obstacle)
            self.cmd_pub.publish(cmd)
            self.publish_state()
            return

        lookahead = self.path[nearest_index]['lookahead']
        lookahead = clamp(lookahead, self.min_lookahead, self.max_lookahead)
        target_index = self.find_target_index(x, y, nearest_index, lookahead)
        target = self.path[target_index]
        steering = self.compute_steering(x, y, yaw, target)
        speed = clamp(target['target_speed'], 0.0, self.max_speed)
        speed = self.apply_obstacle_speed_limit(speed, obstacle)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self.cmd_pub.publish(cmd)
        self.publish_target_marker(target)
        self.publish_state()

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

    def find_nearest_index(self, x, y):
        if self.loop_path:
            candidates = range(len(self.path))
        else:
            start = max(self.path_index - 5, 0)
            stop = min(self.path_index + self.search_window, len(self.path))
            candidates = range(start, stop)

        best_index = self.path_index
        best_distance = float('inf')
        for index in candidates:
            point = self.path[index]
            dist = math.hypot(point['x'] - x, point['y'] - y)
            if dist < best_distance:
                best_distance = dist
                best_index = index
        return best_index, best_distance

    def find_target_index(self, x, y, start_index, lookahead):
        total = len(self.path)
        max_steps = total if self.loop_path else min(self.search_window, total - start_index)
        for offset in range(max_steps):
            index = (start_index + offset) % total
            point = self.path[index]
            if math.hypot(point['x'] - x, point['y'] - y) >= lookahead:
                return index
        return (start_index + max_steps - 1) % total

    def compute_steering(self, x, y, yaw, target):
        dx = target['x'] - x
        dy = target['y'] - y
        target_angle = math.atan2(dy, dx)
        alpha = normalize_angle(target_angle - yaw)
        distance_to_target = max(math.hypot(dx, dy), 1e-3)
        steering = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            distance_to_target,
        )
        steering *= self.steering_gain
        if self.invert_steering:
            steering *= -1.0
        return clamp(steering, -self.steering_limit, self.steering_limit)

    def analyze_scan(self):
        result = {
            'valid': False,
            'front_min': float('inf'),
            'gap_angle': None,
        }
        if self.latest_scan is None:
            return result

        age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
        if age > float(self.get_parameter('scan_timeout').value):
            return result

        scan = self.latest_scan
        half_angle = math.radians(float(self.get_parameter('front_angle_range_deg').value))
        readings = []
        for index, raw_range in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > half_angle:
                continue
            if not math.isfinite(raw_range):
                value = scan.range_max
            else:
                value = clamp(raw_range, scan.range_min, scan.range_max)
            readings.append((angle, value))

        if not readings:
            return result

        front_window = [value for angle, value in readings if abs(angle) < math.radians(15.0)]
        result['valid'] = True
        result['front_min'] = min(front_window) if front_window else min(value for _, value in readings)
        result['gap_angle'] = self.find_gap_angle(readings)
        return result

    def find_gap_angle(self, readings):
        min_distance = float(self.get_parameter('gap_min_distance').value)
        bubble_radius = float(self.get_parameter('gap_bubble_radius').value)
        free = [value >= min_distance for _, value in readings]
        if not any(free):
            return None

        closest_index = min(range(len(readings)), key=lambda i: readings[i][1])
        closest_angle, closest_distance = readings[closest_index]
        bubble_angle = math.atan2(bubble_radius, max(closest_distance, 1e-3))
        for index, (angle, _) in enumerate(readings):
            if abs(angle - closest_angle) <= bubble_angle:
                free[index] = False

        best_start = None
        best_end = None
        current_start = None
        for index, is_free in enumerate(free):
            if is_free and current_start is None:
                current_start = index
            if (not is_free or index == len(free) - 1) and current_start is not None:
                current_end = index if is_free else index - 1
                if best_start is None or current_end - current_start > best_end - best_start:
                    best_start = current_start
                    best_end = current_end
                current_start = None

        if best_start is None:
            return None

        best_index = max(
            range(best_start, best_end + 1),
            key=lambda i: readings[i][1] - abs(readings[i][0]) * 0.15,
        )
        return readings[best_index][0]

    def update_state(self, obstacle, cross_track_error):
        previous = self.state
        emergency_distance = float(self.get_parameter('emergency_stop_distance').value)
        slow_distance = float(self.get_parameter('slow_down_distance').value)
        avoid_enter = float(self.get_parameter('avoidance_enter_distance').value)
        avoid_exit = float(self.get_parameter('avoidance_exit_distance').value)
        return_error = float(self.get_parameter('return_error_threshold').value)

        if not obstacle['valid']:
            self.state = RaceState.FOLLOW_WAYPOINT
        elif obstacle['front_min'] <= emergency_distance:
            self.state = RaceState.EMERGENCY_STOP
        elif self.state == RaceState.AVOID_GAP:
            if obstacle['front_min'] >= avoid_exit and cross_track_error <= return_error:
                self.state = RaceState.FOLLOW_WAYPOINT
        elif obstacle['front_min'] <= avoid_enter and obstacle['gap_angle'] is not None:
            self.state = RaceState.AVOID_GAP
        elif obstacle['front_min'] <= slow_distance:
            self.state = RaceState.SLOW_DOWN
        else:
            self.state = RaceState.FOLLOW_WAYPOINT

        if self.state != previous:
            self.get_logger().info(
                f'State {previous.value} -> {self.state.value}; '
                f'front_min={obstacle["front_min"]:.2f}, '
                f'cross_track_error={cross_track_error:.2f}'
            )

    def apply_obstacle_speed_limit(self, speed, obstacle):
        if self.state == RaceState.SLOW_DOWN and obstacle['valid']:
            slow_distance = float(self.get_parameter('slow_down_distance').value)
            emergency_distance = float(self.get_parameter('emergency_stop_distance').value)
            ratio = (
                (obstacle['front_min'] - emergency_distance)
                / max(slow_distance - emergency_distance, 1e-3)
            )
            limit = self.min_moving_speed + clamp(ratio, 0.0, 1.0) * (
                speed - self.min_moving_speed
            )
            return clamp(limit, self.min_moving_speed, speed)
        return speed

    def make_gap_command(self, obstacle):
        cmd = Twist()
        cmd.linear.x = float(self.get_parameter('avoidance_speed').value)
        steering = obstacle['gap_angle'] * float(self.get_parameter('gap_steering_gain').value)
        if self.invert_steering:
            steering *= -1.0
        cmd.angular.z = clamp(steering, -self.steering_limit, self.steering_limit)
        return cmd

    def update_lap_count(self, nearest_index):
        if len(self.path) < 10:
            return
        wrapped = self.path_index > len(self.path) * 0.8 and nearest_index < len(self.path) * 0.2
        if not wrapped:
            return
        now = self.get_clock().now()
        lap_time = (now - self.last_lap_time).nanoseconds / 1e9
        if lap_time < self.min_lap_time:
            return
        self.lap_count += 1
        self.last_lap_time = now
        self.get_logger().info(f'Lap {self.lap_count} complete: {lap_time:.2f}s')

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def publish_target_marker(self, target):
        if not self.publish_markers:
            return
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'race_waypoint_navigation'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target['x']
        marker.pose.position.y = target['y']
        marker.pose.position.z = 0.05
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.target_marker_pub.publish(marker)

    @staticmethod
    def button_pressed(msg, index):
        return index >= 0 and index < len(msg.buttons) and msg.buttons[index] == 1


def main(args=None):
    rclpy.init(args=args)
    node = RacePurePursuit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
