# #!/usr/bin/env python3

# import math

# import rclpy
# from geometry_msgs.msg import Twist
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan


# class WallFollower(Node):

#     def __init__(self):
#         super().__init__('wall_follower')

#         self.scan_sub = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.scan_callback,
#             10
#         )
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.declare_parameter('target_distance', 0.55)
#         self.declare_parameter('forward_speed', 0.22)
#         self.declare_parameter('slow_speed', 0.08)

#         # Keep angular commands inside the calibrated VESC servo range.
#         self.declare_parameter('turn_speed', 0.18)
#         self.declare_parameter('front_stop_distance', 0.50)

#         self.declare_parameter('kp', 0.45)
#         self.declare_parameter('ki', 0.0)
#         self.declare_parameter('kd', 0.03)
#         self.declare_parameter('max_integral', 0.5)
#         self.declare_parameter('max_angular_speed', 0.25)
#         self.declare_parameter('right_scan_angle_deg', -90.0)

#         self.previous_error = 0.0
#         self.integral = 0.0
#         self.previous_time = self.get_clock().now()

#         self.get_logger().info('wall_follower (NO STOP + FAST RIGHT TURN) started')

#     def scan_callback(self, scan):

#         target_distance = self.get_parameter('target_distance').value
#         forward_speed = self.get_parameter('forward_speed').value
#         slow_speed = self.get_parameter('slow_speed').value
#         turn_speed = self.get_parameter('turn_speed').value

#         kp = self.get_parameter('kp').value
#         ki = self.get_parameter('ki').value
#         kd = self.get_parameter('kd').value

#         max_integral = self.get_parameter('max_integral').value
#         max_angular_speed = self.get_parameter('max_angular_speed').value

#         front_stop_distance = self.get_parameter('front_stop_distance').value
#         right_scan_angle_deg = self.get_parameter('right_scan_angle_deg').value

#         # ROS LaserScan angles are positive to the left and negative to the right.
#         right = self.get_range(scan, right_scan_angle_deg)
#         front = self.get_range(scan, 0.0)

#         cmd = Twist()
#         mode = 'follow'

#         # 🚀 เจอของด้านหน้า → วิ่ง + เลี้ยวขวา (ไม่หยุด)
#         if front < front_stop_distance:
#             self.reset_pid()

#             cmd.linear.x = turn_speed             # 🔥 วิ่งต่อ
#             cmd.angular.z = -max_angular_speed    # 🔥 หักขวาแรง

#             mode = 'avoid_fast'
#             self.cmd_pub.publish(cmd)
#             self.log_command(mode, right, front, cmd)
#             return

#         # 🧱 ไม่เจอกำแพงขวา
#         if math.isinf(right):
#             self.reset_pid()
#             cmd.linear.x = slow_speed
#             cmd.angular.z = 0.4
#             mode = 'search_wall'
#             self.cmd_pub.publish(cmd)
#             self.log_command(mode, right, front, cmd)
#             return

#         # 🎯 PID เกาะผนังขวา
#         error = target_distance - right
#         angular = self.calculate_pid(error, kp, ki, kd, max_integral)

#         cmd.linear.x = forward_speed
#         cmd.angular.z = self.clamp(angular, -max_angular_speed, max_angular_speed)

#         self.cmd_pub.publish(cmd)
#         self.log_command(mode, right, front, cmd)

#     def calculate_pid(self, error, kp, ki, kd, max_integral):

#         current_time = self.get_clock().now()
#         dt = (current_time - self.previous_time).nanoseconds / 1e9

#         if dt <= 0.0:
#             dt = 1e-3

#         self.integral += error * dt
#         self.integral = self.clamp(self.integral, -max_integral, max_integral)

#         derivative = (error - self.previous_error) / dt

#         self.previous_error = error
#         self.previous_time = current_time

#         return (kp * error) + (ki * self.integral) + (kd * derivative)

#     def reset_pid(self):
#         self.previous_error = 0.0
#         self.integral = 0.0
#         self.previous_time = self.get_clock().now()

#     def log_command(self, mode, right, front, cmd):
#         self.get_logger().info(
#             'mode=%s right=%.2f front=%.2f linear=%.2f angular=%.2f' % (
#                 mode,
#                 right,
#                 front,
#                 cmd.linear.x,
#                 cmd.angular.z,
#             ),
#             throttle_duration_sec=1.0
#         )

#     def get_range(self, scan, angle_deg):
#         angle_rad = math.radians(angle_deg)
#         index = int(round((angle_rad - scan.angle_min) / scan.angle_increment))

#         if index < 0 or index >= len(scan.ranges):
#             return float('inf')

#         value = scan.ranges[index]

#         if math.isnan(value):
#             return float('inf')

#         if value < scan.range_min:
#             return scan.range_min

#         if value > scan.range_max:
#             return float('inf')

#         return value

#     @staticmethod
#     def clamp(value, lower, upper):
#         return max(lower, min(upper, value))


# def main(args=None):
#     rclpy.init(args=args)
#     node = WallFollower()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cmd_pub.publish(Twist())
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter('target_distance', 0.55)   # ระยะกึ่งกลางเลน (ครึ่งหนึ่งของความกว้างเลน)
        self.declare_parameter('forward_speed', 0.22)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('turn_speed', 0.18)
        self.declare_parameter('front_stop_distance', 0.50)

        self.declare_parameter('kp', 0.45)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.03)
        self.declare_parameter('max_integral', 0.5)
        self.declare_parameter('max_angular_speed', 0.25)

        self.declare_parameter('right_scan_angle_deg', -90.0)
        self.declare_parameter('left_scan_angle_deg', 90.0)

        # ระยะ LiDAR เยื้องจากกึ่งกลางรถ (บวก = เยื้องซ้าย, ลบ = เยื้องขวา)
        self.declare_parameter('lidar_offset_m', 0.05)

        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = self.get_clock().now()

        self.get_logger().info('wall_follower (LANE CENTER + LIDAR OFFSET) started')

    def scan_callback(self, scan):

        forward_speed = self.get_parameter('forward_speed').value
        slow_speed    = self.get_parameter('slow_speed').value
        turn_speed    = self.get_parameter('turn_speed').value

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value

        max_integral     = self.get_parameter('max_integral').value
        max_angular_speed = self.get_parameter('max_angular_speed').value

        front_stop_distance  = self.get_parameter('front_stop_distance').value
        right_scan_angle_deg = self.get_parameter('right_scan_angle_deg').value
        left_scan_angle_deg  = self.get_parameter('left_scan_angle_deg').value
        lidar_offset_m       = self.get_parameter('lidar_offset_m').value

        right = self.get_range(scan, right_scan_angle_deg)
        left  = self.get_range(scan, left_scan_angle_deg)
        front = self.get_range(scan, 0.0)

        cmd = Twist()
        mode = 'center'

        # 🚀 เจอของด้านหน้า → วิ่ง + เลี้ยวขวา (ไม่หยุด)
        if front < front_stop_distance:
            self.reset_pid()
            cmd.linear.x  = turn_speed
            cmd.angular.z = -max_angular_speed
            mode = 'avoid_fast'
            self.cmd_pub.publish(cmd)
            self.log_command(mode, left, right, front, cmd)
            return

        # 🧱 เจอผนังข้างเดียว → fallback เกาะผนังนั้น
        left_valid  = not math.isinf(left)
        right_valid = not math.isinf(right)

        if not left_valid and not right_valid:
            # ไม่เจอผนังเลย → ไปตรงๆ ช้าๆ
            self.reset_pid()
            cmd.linear.x  = slow_speed
            cmd.angular.z = 0.0
            mode = 'no_wall'
            self.cmd_pub.publish(cmd)
            self.log_command(mode, left, right, front, cmd)
            return

        if not left_valid:
            # เห็นแค่ผนังขวา → เกาะขวา (mode เดิม)
            target_distance = self.get_parameter('target_distance').value
            # ระยะแท้จริงของรถถึงผนังขวา
            true_right = right - lidar_offset_m
            error = target_distance - true_right

        elif not right_valid:
            # เห็นแค่ผนังซ้าย → เกาะซ้าย
            target_distance = self.get_parameter('target_distance').value
            true_left = left + lidar_offset_m
            error = true_left - target_distance

        else:
            # 🎯 เห็นทั้งสองผนัง → อยู่กึ่งกลาง
            # ชดเชย offset: ระยะแท้จริงจากกึ่งกลางรถ
            true_left  = left  + lidar_offset_m   # LiDAR ใกล้ผนังซ้ายกว่ารถจริง
            true_right = right - lidar_offset_m   # LiDAR ไกลผนังขวากว่ารถจริง

            # error > 0 → รถเอียงขวา → ต้องเลี้ยวซ้าย
            # error < 0 → รถเอียงซ้าย → ต้องเลี้ยวขวา
            error = (true_left - true_right) / 2.0
            mode = 'center'

        angular = self.calculate_pid(error, kp, ki, kd, max_integral)

        cmd.linear.x  = forward_speed
        cmd.angular.z = self.clamp(angular, -max_angular_speed, max_angular_speed)

        self.cmd_pub.publish(cmd)
        self.log_command(mode, left, right, front, cmd)

    def calculate_pid(self, error, kp, ki, kd, max_integral):

        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9

        if dt <= 0.0:
            dt = 1e-3

        self.integral += error * dt
        self.integral = self.clamp(self.integral, -max_integral, max_integral)

        derivative = (error - self.previous_error) / dt

        self.previous_error = error
        self.previous_time  = current_time

        return (kp * error) + (ki * self.integral) + (kd * derivative)

    def reset_pid(self):
        self.previous_error = 0.0
        self.integral       = 0.0
        self.previous_time  = self.get_clock().now()

    def log_command(self, mode, left, right, front, cmd):
        self.get_logger().info(
            'mode=%s left=%.2f right=%.2f front=%.2f linear=%.2f angular=%.2f' % (
                mode, left, right, front,
                cmd.linear.x, cmd.angular.z,
            ),
            throttle_duration_sec=1.0
        )

    def get_range(self, scan, angle_deg):
        angle_rad = math.radians(angle_deg)
        index = int(round((angle_rad - scan.angle_min) / scan.angle_increment))

        if index < 0 or index >= len(scan.ranges):
            return float('inf')

        value = scan.ranges[index]

        if math.isnan(value):
            return float('inf')

        if value < scan.range_min:
            return scan.range_min

        if value > scan.range_max:
            return float('inf')

        return value

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

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