import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import math


class RightWallAvoid(Node):
    def __init__(self):
        super().__init__('right_wall_avoid')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # ===== ปรับค่าตรงนี้ตามรถจริง =====
        self.normal_speed = 0.5          # m/s
        self.slow_speed = 0.25           # m/s

        self.right_wall_threshold = 0.60 # m
        self.front_wall_threshold = 0.50 # m

        self.left_steer = 0.35           # rad เลี้ยวซ้าย
        self.right_steer = -0.25         # rad เลี้ยวขวา
        self.straight = 0.0

        self.max_steer = 0.40            # จำกัดมุมเลี้ยว ไม่ให้ servo หักเกิน

    def get_range_in_angle(self, scan, angle_min_deg, angle_max_deg):
        """
        หาค่าระยะต่ำสุดในช่วงมุมที่ต้องการ
        angle ใช้หน่วย degree เพื่อให้อ่านง่าย
        """

        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)

        ranges = []

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if angle_min <= angle <= angle_max:
                if math.isfinite(distance):
                    if scan.range_min < distance < scan.range_max:
                        ranges.append(distance)

        if len(ranges) == 0:
            return float('inf')

        return min(ranges)

    def scan_callback(self, scan):
        # ด้านหน้า: -15 ถึง +15 องศา
        front_dist = self.get_range_in_angle(scan, -15, 15)
    
        # ด้านขวา: -90 ถึง -30 องศา
        right_dist = self.get_range_in_angle(scan, -90, -30)
        self.get_logger().info(
    f"FRONT={front_dist:.2f} RIGHT={right_dist:.2f}"
)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'

        # ===== Logic หลัก =====
        if front_dist < self.front_wall_threshold:
            # ถ้าข้างหน้าตัน ให้ชะลอและเลี้ยวซ้ายแรง
            speed = self.slow_speed
            steer = self.left_steer

            self.get_logger().info(
                f'FRONT WALL! front={front_dist:.2f} m -> turn LEFT'
            )

        elif right_dist < self.right_wall_threshold:
            # ถ้าด้านขวาเจอกำแพงใกล้ ให้เลี้ยวซ้าย
            speed = self.normal_speed
            steer = self.left_steer

            self.get_logger().info(
                f'RIGHT WALL! right={right_dist:.2f} m -> turn LEFT'
            )

        else:
            # ถ้าไม่มีอะไรใกล้มาก ให้วิ่งตรง
            speed = self.normal_speed
            steer = self.straight

            self.get_logger().info(
                f'CLEAR right={right_dist:.2f} m front={front_dist:.2f} m -> straight'
            )

        # จำกัดมุมเลี้ยว
        steer = max(-self.max_steer, min(self.max_steer, steer))

        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RightWallAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
