#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class FollowTheGap(Node):

    def __init__(self):
        super().__init__('follow_the_gap')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/follow_the_gap/markers',
            10
        )

        self.declare_parameter(
            'scan_angle_min_deg',
            -90.0
        )  # มุมซ้ายสุดที่ใช้สแกนด้านหน้า หน่วยองศา
        self.declare_parameter(
            'scan_angle_max_deg',
            90.0
        )  # มุมขวาสุดที่ใช้สแกนด้านหน้า หน่วยองศา
        self.declare_parameter(
            'max_lidar_range',
            3.5
        )  # จำกัดระยะไกลสุดที่นำมาคิด เพื่อไม่ให้ค่าระยะไกลเกินแกว่ง
        self.declare_parameter(
            'min_lidar_range',
            0.10
        )  # ระยะใกล้สุดที่ยอมรับจาก LiDAR ต่ำกว่านี้จะถือว่าอ่านไม่ได้
        self.declare_parameter(
            'bubble_radius',
            0.1
        )  # รัศมีวง safety bubble รอบ obstacle แต่ละจุด หน่วยเมตร
        self.declare_parameter(
            'bubble_obstacle_count',
            3
        )  # จำนวน obstacle ที่ใกล้ที่สุดที่จะสร้าง bubble รอบจุดนั้น
        self.declare_parameter(
            'max_bubble_markers',
            3
        )  # จำกัดจำนวน bubble ที่วาดใน RViz2, 0 คือวาดทุกจุดที่เลือก
        self.declare_parameter(
            'free_space_threshold',
            0.3
        )  # ระยะขั้นต่ำที่ถือว่าจุดนั้นเป็นพื้นที่ว่าง หน่วยเมตร
        self.declare_parameter(
            'car_width',
            0.25
        )  # ความกว้างตัวรถโดยประมาณ ใช้เช็คว่า gap กว้างพอไหม
        self.declare_parameter(
            'safety_margin',
            0.12
        )  # ระยะเผื่อซ้ายขวาของตัวรถตอนเช็คความกว้าง gap
        self.declare_parameter(
            'min_gap_points',
            5
        )  # จำนวนจุด LiDAR ขั้นต่ำที่ต้องต่อกัน จึงจะนับเป็น gap
        self.declare_parameter(
            'best_point_depth_ratio',
            0.30
        )  # เลือกบริเวณที่ลึกอย่างน้อยกี่เปอร์เซ็นต์ของจุดลึกสุดใน gap
        self.declare_parameter(
            'smoothing_window',
            3
        )  # จำนวนจุดที่ใช้เฉลี่ยค่า range เพื่อลด noise

        self.declare_parameter(
            'forward_speed',
            0.30
        )  # ความเร็วปกติเมื่อทางโล่ง หน่วย m/s
        self.declare_parameter(
            'slow_speed',
            0.25
        )  # ความเร็วช้าเมื่อ gap แคบหรือเป้าหมายอยู่ใกล้ หน่วย m/s
        self.declare_parameter(
            'min_speed',
            0.20
        )  # ความเร็วต่ำสุดที่อนุญาตเมื่อยังไม่ต้องหยุด หน่วย m/s
        self.declare_parameter(
            'slow_down_distance',
            0.65
        )  # ถ้าเป้าหมายอยู่ใกล้กว่าระยะนี้ ให้ลดความเร็ว
        self.declare_parameter(
            'emergency_stop_distance',
            0.50
        )  # ถ้าหน้ารถใกล้สิ่งกีดขวางกว่านี้ ให้หยุด
        self.declare_parameter(
            'front_check_angle_deg',
            15.0
        )  # ครึ่งมุมด้านหน้าที่ใช้เช็คหยุดฉุกเฉิน เช่น 15 คือ -15 ถึง +15

        self.declare_parameter(
            'kp',
            1.5
        )  # gain สัดส่วนของ PD ยิ่งมากยิ่งหักเลี้ยวไว
        self.declare_parameter(
            'kd',
            0.3
        )  # gain อนุพันธ์ของ PD ช่วยลดการส่ายและตอบสนองต่อการเปลี่ยนเร็ว
        self.declare_parameter(
            'max_angular_speed',
            1.5
        )  # จำกัดคำสั่งเลี้ยวสูงสุดที่ส่งออกทาง cmd_vel.angular.z
        self.declare_parameter(
            'invert_steering',
            False
        )  # กลับทิศเลี้ยว ถ้า driver รถรับซ้ายขวาสลับกับ ROS
        self.declare_parameter(
            'publish_visualization',
            True
        )  # เปิด/ปิด marker สำหรับโชว์ gap, bubble, target ใน RViz2
        self.declare_parameter(
            'marker_lifetime',
            0.25
        )  # อายุ marker ใน RViz2 หน่วยวินาที ควรสั้นกว่าคาบอัปเดตเล็กน้อย

        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

        self.get_logger().info('follow_the_gap started')

    def scan_callback(self, scan):
        params = self.get_params()

        angles, ranges = self.get_front_scan(scan, params)
        cmd = Twist()

        if not angles:
            self.reset_pd()
            self.cmd_pub.publish(cmd)
            self.get_logger().warn('No usable LIDAR points in front scan')
            return

        ranges = self.smooth_ranges(ranges, params['smoothing_window'])
        front_check_angle = params['front_check_angle_deg']
        front = self.get_min_range_between(
            angles,
            ranges,
            -front_check_angle,
            front_check_angle
        )

        obstacle_indices = self.find_obstacle_indices(
            ranges,
            params['max_lidar_range']
        )
        closest_index = self.find_closest_index(ranges)
        if closest_index is None:
            closest_range = 0.0
        else:
            closest_range = ranges[closest_index]

        bubble_indices = self.select_nearest_indices(
            obstacle_indices,
            ranges,
            params['bubble_obstacle_count']
        )

        bubble_ranges = self.apply_safety_bubbles(
            angles,
            ranges,
            bubble_indices,
            params['bubble_radius']
        )

        gaps = self.find_gaps(angles, bubble_ranges, params)
        passable_gaps = [gap for gap in gaps if gap['passable']]

        if passable_gaps:
            selected_gap = self.choose_best_gap(passable_gaps)
            mode = 'gap'
        elif gaps:
            selected_gap = self.choose_best_gap(gaps)
            mode = 'narrow_gap'
        else:
            selected_gap = None
            mode = 'blocked'

        target_index = None
        if selected_gap is None:
            target_angle = self.choose_fallback_angle(angles, ranges)
            target_range = front
            gap_width = 0.0
        else:
            target_index = self.choose_best_point(
                bubble_ranges,
                selected_gap,
                params['best_point_depth_ratio']
            )
            target_angle = angles[target_index]
            target_range = bubble_ranges[target_index]
            gap_width = selected_gap['width_m']

        error = target_angle
        angular = self.calculate_pd(error, params['kp'], params['kd'])
        steering_sign = -1.0 if params['invert_steering'] else 1.0

        cmd.angular.z = self.clamp(
            angular * steering_sign,
            -params['max_angular_speed'],
            params['max_angular_speed']
        )
        cmd.linear.x = self.calculate_speed(
            mode,
            target_angle,
            target_range,
            front,
            params
        )

        self.cmd_pub.publish(cmd)
        self.publish_visualization(
            scan,
            angles,
            ranges,
            bubble_ranges,
            bubble_indices,
            selected_gap,
            target_index,
            target_angle,
            target_range,
            mode,
            params
        )
        self.log_command(
            mode,
            math.degrees(target_angle),
            target_range,
            closest_range,
            gap_width,
            front,
            cmd
        )

    def get_params(self):
        return {
            'scan_angle_min_deg': float(
                self.get_parameter('scan_angle_min_deg').value
            ),
            'scan_angle_max_deg': float(
                self.get_parameter('scan_angle_max_deg').value
            ),
            'max_lidar_range': float(
                self.get_parameter('max_lidar_range').value
            ),
            'min_lidar_range': float(
                self.get_parameter('min_lidar_range').value
            ),
            'bubble_radius': float(self.get_parameter('bubble_radius').value),
            'bubble_obstacle_count': int(
                self.get_parameter('bubble_obstacle_count').value
            ),
            'max_bubble_markers': int(
                self.get_parameter('max_bubble_markers').value
            ),
            'free_space_threshold': float(
                self.get_parameter('free_space_threshold').value
            ),
            'car_width': float(self.get_parameter('car_width').value),
            'safety_margin': float(self.get_parameter('safety_margin').value),
            'min_gap_points': int(self.get_parameter('min_gap_points').value),
            'best_point_depth_ratio': float(
                self.get_parameter('best_point_depth_ratio').value
            ),
            'smoothing_window': int(
                self.get_parameter('smoothing_window').value
            ),
            'forward_speed': float(self.get_parameter('forward_speed').value),
            'slow_speed': float(self.get_parameter('slow_speed').value),
            'min_speed': float(self.get_parameter('min_speed').value),
            'slow_down_distance': float(
                self.get_parameter('slow_down_distance').value
            ),
            'emergency_stop_distance': float(
                self.get_parameter('emergency_stop_distance').value
            ),
            'front_check_angle_deg': float(
                self.get_parameter('front_check_angle_deg').value
            ),
            'kp': float(self.get_parameter('kp').value),
            'kd': float(self.get_parameter('kd').value),
            'max_angular_speed': float(
                self.get_parameter('max_angular_speed').value
            ),
            'invert_steering': bool(
                self.get_parameter('invert_steering').value
            ),
            'publish_visualization': bool(
                self.get_parameter('publish_visualization').value
            ),
            'marker_lifetime': float(
                self.get_parameter('marker_lifetime').value
            ),
        }

    def get_front_scan(self, scan, params):
        min_angle = math.radians(params['scan_angle_min_deg'])
        max_angle = math.radians(params['scan_angle_max_deg'])

        if min_angle > max_angle:
            min_angle, max_angle = max_angle, min_angle

        angles = []
        ranges = []

        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if min_angle <= angle <= max_angle:
                angles.append(angle)
                ranges.append(self.clean_range(value, scan, params))

        return angles, ranges

    def clean_range(self, value, scan, params):
        usable_max = params['max_lidar_range']

        if math.isfinite(scan.range_max):
            usable_max = min(usable_max, scan.range_max)

        if not math.isfinite(value):
            return usable_max

        if value < params['min_lidar_range']:
            return 0.0

        return self.clamp(value, 0.0, usable_max)

    def smooth_ranges(self, ranges, window_size):
        if window_size <= 1 or len(ranges) < 3:
            return list(ranges)

        if window_size % 2 == 0:
            window_size += 1

        half_window = window_size // 2
        smoothed = []

        for index in range(len(ranges)):
            start = max(0, index - half_window)
            end = min(len(ranges), index + half_window + 1)
            window = ranges[start:end]
            smoothed.append(sum(window) / len(window))

        return smoothed

    def find_closest_index(self, ranges):
        best_index = None
        best_range = float('inf')

        for index, value in enumerate(ranges):
            if 0.0 < value < best_range:
                best_index = index
                best_range = value

        return best_index

    def find_obstacle_indices(self, ranges, max_lidar_range):
        obstacle_indices = []

        for index, value in enumerate(ranges):
            if 0.0 < value < max_lidar_range:
                obstacle_indices.append(index)

        return obstacle_indices

    def select_nearest_indices(self, indices, ranges, count):
        if count <= 0:
            return indices

        sorted_indices = sorted(indices, key=lambda index: ranges[index])
        return sorted_indices[:count]

    def apply_safety_bubbles(
        self,
        angles,
        ranges,
        obstacle_indices,
        bubble_radius
    ):
        bubble_ranges = list(ranges)

        if not obstacle_indices or bubble_radius <= 0.0:
            return bubble_ranges

        points = [
            self.polar_to_xy(angle, distance)
            for angle, distance in zip(angles, ranges)
        ]

        for obstacle_index in obstacle_indices:
            self.apply_safety_bubble_to_index(
                angles,
                ranges,
                bubble_ranges,
                points,
                obstacle_index,
                bubble_radius
            )

        return bubble_ranges

    def apply_safety_bubble_to_index(
        self,
        angles,
        ranges,
        bubble_ranges,
        points,
        obstacle_index,
        bubble_radius
    ):
        obstacle_range = ranges[obstacle_index]

        if obstacle_range <= 0.0:
            return

        obstacle_angle = angles[obstacle_index]
        obstacle_x, obstacle_y = points[obstacle_index]
        ratio = self.clamp(bubble_radius / obstacle_range, 0.0, 1.0)
        bubble_angle = math.asin(ratio)

        for index, value in enumerate(ranges):
            angle = angles[index]
            point_x, point_y = points[index]
            distance_to_obstacle = math.hypot(
                point_x - obstacle_x,
                point_y - obstacle_y
            )

            if (
                abs(angle - obstacle_angle) <= bubble_angle
                or distance_to_obstacle <= bubble_radius
            ):
                bubble_ranges[index] = 0.0

    def find_gaps(self, angles, ranges, params):
        gaps = []
        start = None
        threshold = params['free_space_threshold']

        for index, value in enumerate(ranges):
            if value >= threshold:
                if start is None:
                    start = index
            elif start is not None:
                self.add_gap(gaps, angles, ranges, start, index - 1, params)
                start = None

        if start is not None:
            self.add_gap(gaps, angles, ranges, start, len(ranges) - 1, params)

        return gaps

    def add_gap(self, gaps, angles, ranges, start, end, params):
        point_count = end - start + 1

        if point_count < params['min_gap_points']:
            return

        gap_ranges = ranges[start:end + 1]
        angle_width = self.get_gap_angle_width(angles, start, end)
        depth_for_width = self.lower_quartile(gap_ranges)
        width_m = 2.0 * depth_for_width * math.sin(angle_width / 2.0)
        required_width = params['car_width'] + (2.0 * params['safety_margin'])

        gaps.append({
            'start': start,
            'end': end,
            'point_count': point_count,
            'angle_width': angle_width,
            'width_m': width_m,
            'max_depth': max(gap_ranges),
            'mean_depth': sum(gap_ranges) / len(gap_ranges),
            'center_angle': (angles[start] + angles[end]) / 2.0,
            'passable': width_m >= required_width,
        })

    def get_gap_angle_width(self, angles, start, end):
        if len(angles) < 2:
            return 0.0

        angle_step = abs(angles[1] - angles[0])
        return abs(angles[end] - angles[start]) + angle_step

    def choose_best_gap(self, gaps):
        return max(
            gaps,
            key=lambda gap: (
                gap['max_depth'],
                gap['mean_depth'],
                gap['width_m'],
                -abs(gap['center_angle']),
            )
        )

    def choose_best_point(self, ranges, gap, depth_ratio):
        start = gap['start']
        end = gap['end']
        gap_ranges = ranges[start:end + 1]
        deepest_range = max(gap_ranges)
        deep_threshold = deepest_range * self.clamp(depth_ratio, 0.0, 1.0)

        best_run_start = None
        best_run_end = None
        run_start = None

        for index in range(start, end + 1):
            if ranges[index] >= deep_threshold:
                if run_start is None:
                    run_start = index
            elif run_start is not None:
                best_run_start, best_run_end = self.choose_longer_run(
                    best_run_start,
                    best_run_end,
                    run_start,
                    index - 1
                )
                run_start = None

        if run_start is not None:
            best_run_start, best_run_end = self.choose_longer_run(
                best_run_start,
                best_run_end,
                run_start,
                end
            )

        if best_run_start is None or best_run_end is None:
            return (start + end) // 2

        return (best_run_start + best_run_end) // 2

    def choose_longer_run(self, best_start, best_end, run_start, run_end):
        if best_start is None or best_end is None:
            return run_start, run_end

        best_length = best_end - best_start + 1
        run_length = run_end - run_start + 1

        if run_length > best_length:
            return run_start, run_end

        return best_start, best_end

    def choose_fallback_angle(self, angles, ranges):
        left_values = [
            value for angle, value in zip(angles, ranges)
            if angle >= 0.0
        ]
        right_values = [
            value for angle, value in zip(angles, ranges)
            if angle < 0.0
        ]

        left_score = self.average(left_values)
        right_score = self.average(right_values)

        if left_score >= right_score:
            return math.radians(45.0)

        return math.radians(-45.0)

    def calculate_pd(self, error, kp, kd):
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 1e-3

        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        self.previous_time = current_time

        return (kp * error) + (kd * derivative)

    def calculate_speed(
        self,
        mode,
        target_angle,
        target_range,
        front,
        params
    ):
        if mode == 'blocked' or front < params['emergency_stop_distance']:
            return 0.0

        steering_ratio = min(abs(target_angle) / math.radians(90.0), 1.0)
        speed = params['forward_speed'] - (
            params['forward_speed'] - params['slow_speed']
        ) * steering_ratio

        if target_range < params['slow_down_distance']:
            speed = min(speed, params['slow_speed'])

        if mode == 'narrow_gap':
            speed = min(speed, params['slow_speed'])

        return self.clamp(speed, params['min_speed'], params['forward_speed'])

    def publish_visualization(
        self,
        scan,
        angles,
        ranges,
        bubble_ranges,
        bubble_indices,
        selected_gap,
        target_index,
        target_angle,
        target_range,
        mode,
        params
    ):
        if not params['publish_visualization']:
            return

        header = scan.header
        if not header.frame_id:
            header.frame_id = 'base_link'

        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        marker_id = 0

        bubble_marker_indices = self.limit_bubble_marker_indices(
            bubble_indices,
            ranges,
            params['max_bubble_markers']
        )

        for bubble_index in bubble_marker_indices:
            marker_id = self.add_obstacle_markers(
                markers,
                header,
                marker_id,
                angles[bubble_index],
                ranges[bubble_index],
                params
            )

        if selected_gap is not None:
            marker_id = self.add_gap_markers(
                markers,
                header,
                marker_id,
                angles,
                bubble_ranges,
                selected_gap,
                params
            )

        marker_id = self.add_target_markers(
            markers,
            header,
            marker_id,
            target_angle,
            target_range,
            target_index,
            params
        )
        self.add_status_marker(
            markers,
            header,
            marker_id,
            mode,
            target_angle,
            target_range,
            params
        )

        self.marker_pub.publish(markers)

    def limit_bubble_marker_indices(self, indices, ranges, max_marker_count):
        if max_marker_count <= 0 or len(indices) <= max_marker_count:
            return indices

        sorted_indices = sorted(indices, key=lambda index: ranges[index])
        return sorted_indices[:max_marker_count]

    def add_obstacle_markers(
        self,
        markers,
        header,
        marker_id,
        obstacle_angle,
        obstacle_range,
        params
    ):
        obstacle_point = self.polar_to_point(obstacle_angle, obstacle_range)
        bubble_radius = params['bubble_radius']

        bubble = self.make_marker(
            header,
            marker_id,
            'safety_bubble',
            Marker.CYLINDER,
            (1.0, 0.05, 0.05, 0.22),
            (bubble_radius * 2.0, bubble_radius * 2.0, 0.03),
            params
        )
        bubble.pose.position = obstacle_point
        bubble.pose.position.z = 0.015
        markers.markers.append(bubble)
        marker_id += 1

        obstacle = self.make_marker(
            header,
            marker_id,
            'closest_obstacle',
            Marker.SPHERE,
            (1.0, 0.0, 0.0, 0.9),
            (0.12, 0.12, 0.12),
            params
        )
        obstacle.pose.position = obstacle_point
        obstacle.pose.position.z = 0.08
        markers.markers.append(obstacle)
        return marker_id + 1

    def add_gap_markers(
        self,
        markers,
        header,
        marker_id,
        angles,
        bubble_ranges,
        selected_gap,
        params
    ):
        start = selected_gap['start']
        end = selected_gap['end']

        boundaries = self.make_marker(
            header,
            marker_id,
            'selected_gap_boundaries',
            Marker.LINE_LIST,
            (1.0, 0.8, 0.05, 0.9),
            (0.035, 0.0, 0.0),
            params
        )
        start_point = self.polar_to_point(angles[start], bubble_ranges[start])
        end_point = self.polar_to_point(angles[end], bubble_ranges[end])
        boundaries.points = [
            Point(x=0.0, y=0.0, z=0.04),
            Point(x=start_point.x, y=start_point.y, z=0.04),
            Point(x=0.0, y=0.0, z=0.04),
            Point(x=end_point.x, y=end_point.y, z=0.04),
        ]
        markers.markers.append(boundaries)
        marker_id += 1

        arc = self.make_marker(
            header,
            marker_id,
            'selected_gap_arc',
            Marker.LINE_STRIP,
            (0.05, 0.9, 0.2, 0.9),
            (0.04, 0.0, 0.0),
            params
        )
        step = max(1, (end - start) // 30)
        arc.points = []

        for index in range(start, end + 1, step):
            point = self.polar_to_point(angles[index], bubble_ranges[index])
            arc.points.append(Point(x=point.x, y=point.y, z=0.06))

        if arc.points[-1].x != end_point.x or arc.points[-1].y != end_point.y:
            arc.points.append(Point(x=end_point.x, y=end_point.y, z=0.06))

        markers.markers.append(arc)
        return marker_id + 1

    def add_target_markers(
        self,
        markers,
        header,
        marker_id,
        target_angle,
        target_range,
        target_index,
        params
    ):
        usable_range = self.get_marker_range(target_range, params)
        target_point = self.polar_to_point(target_angle, usable_range)

        ray = self.make_marker(
            header,
            marker_id,
            'target_direction',
            Marker.ARROW,
            (0.05, 0.35, 1.0, 0.95),
            (0.06, 0.12, 0.12),
            params
        )
        ray.points = [
            Point(x=0.0, y=0.0, z=0.10),
            Point(x=target_point.x, y=target_point.y, z=0.10),
        ]
        markers.markers.append(ray)
        marker_id += 1

        target = self.make_marker(
            header,
            marker_id,
            'target_point',
            Marker.SPHERE,
            (0.05, 0.35, 1.0, 1.0),
            (0.16, 0.16, 0.16),
            params
        )
        target.pose.position = target_point
        target.pose.position.z = 0.12
        markers.markers.append(target)

        return marker_id + 1

    def add_status_marker(
        self,
        markers,
        header,
        marker_id,
        mode,
        target_angle,
        target_range,
        params
    ):
        status = self.make_marker(
            header,
            marker_id,
            'follow_the_gap_status',
            Marker.TEXT_VIEW_FACING,
            (1.0, 1.0, 1.0, 0.95),
            (0.0, 0.0, 0.24),
            params
        )
        status.pose.position.x = 0.0
        status.pose.position.y = 0.0
        status.pose.position.z = 0.45
        status.text = 'FTG %s  %.1f deg  %.2f m' % (
            mode,
            math.degrees(target_angle),
            self.get_marker_range(target_range, params)
        )
        markers.markers.append(status)

    def make_marker(
        self,
        header,
        marker_id,
        namespace,
        marker_type,
        color,
        scale,
        params
    ):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = Duration(
            seconds=params['marker_lifetime']
        ).to_msg()
        return marker

    def polar_to_point(self, angle, distance):
        return Point(
            x=distance * math.cos(angle),
            y=distance * math.sin(angle),
            z=0.0
        )

    def polar_to_xy(self, angle, distance):
        return (
            distance * math.cos(angle),
            distance * math.sin(angle)
        )

    def get_marker_range(self, value, params):
        if math.isfinite(value) and value > 0.0:
            return min(value, params['max_lidar_range'])

        return min(1.0, params['max_lidar_range'])

    def reset_pd(self):
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

    def get_min_range_between(self, angles, ranges, min_deg, max_deg):
        min_angle = math.radians(min_deg)
        max_angle = math.radians(max_deg)
        values = [
            value for angle, value in zip(angles, ranges)
            if min_angle <= angle <= max_angle and value > 0.0
        ]

        if not values:
            return float('inf')

        return min(values)

    def log_command(
        self,
        mode,
        target_angle_deg,
        target_range,
        closest_range,
        gap_width,
        front,
        cmd
    ):
        self.get_logger().info(
            (
                'mode=%s target=%.1fdeg target_range=%.2f '
                'closest=%.2f gap_width=%.2f front=%.2f '
                'linear=%.2f angular=%.2f'
            ) % (
                mode,
                target_angle_deg,
                target_range,
                closest_range,
                gap_width,
                front,
                cmd.linear.x,
                cmd.angular.z,
            ),
            throttle_duration_sec=0.5
        )

    @staticmethod
    def lower_quartile(values):
        ordered = sorted(values)
        index = max(0, len(ordered) // 4)
        return ordered[index]

    @staticmethod
    def average(values):
        if not values:
            return 0.0

        return sum(values) / len(values)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
