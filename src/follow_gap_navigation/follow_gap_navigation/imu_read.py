#!/usr/bin/env python3
import time
import math

import smbus2 as smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ICM20948Publisher(Node):
    def __init__(self):
        super().__init__('icm20948_publisher')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('imu_addr', 0x68)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)

        self.bus_id = int(self.get_parameter('i2c_bus').value)
        self.imu_addr = int(self.get_parameter('imu_addr').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bus = smbus.SMBus(self.bus_id)

        self.GYRO_Z_DEADBAND_DPS = 0.3
        self.MAX_DT_SECONDS = 0.1

        self.yaw = 0.0
        self.prev_time = time.monotonic()

        self.acc_pitch_offset = 0.0
        self.acc_roll_offset = 0.0
        self.gyro_x_offset = 0.0
        self.gyro_y_offset = 0.0
        self.gyro_z_offset = 0.0

        self.imu_pub = self.create_publisher(Imu, self.get_parameter('imu_topic').value, 10)

        self.init_imu()
        self.calibrate_imu()

        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(f'ICM20948 publishing to {self.get_parameter("imu_topic").value}')

    def select_bank(self, bank):
        self.bus.write_byte_data(self.imu_addr, 0x7F, bank << 4)

    def init_imu(self):
        self.get_logger().info('Initializing IMU...')

        self.select_bank(0)

        self.bus.write_byte_data(self.imu_addr, 0x06, 0x80)
        time.sleep(0.1)

        self.bus.write_byte_data(self.imu_addr, 0x06, 0x01)
        self.bus.write_byte_data(self.imu_addr, 0x07, 0x00)

        self.select_bank(2)

        self.bus.write_byte_data(self.imu_addr, 0x01, 0x04)
        self.bus.write_byte_data(self.imu_addr, 0x14, 0x04)

        self.select_bank(0)

        self.get_logger().info('IMU Ready')

    def read_word(self, reg):
        high = self.bus.read_byte_data(self.imu_addr, reg)
        low = self.bus.read_byte_data(self.imu_addr, reg + 1)

        value = (high << 8) | low

        if value >= 32768:
            value -= 65536

        return value

    def read_accel(self):
        self.select_bank(0)

        ax = self.read_word(0x2D)
        ay = self.read_word(0x2F)
        az = self.read_word(0x31)

        ax = ax / 8192.0
        ay = ay / 8192.0
        az = az / 8192.0

        return ax, ay, az

    def read_gyro(self):
        self.select_bank(0)

        gx = self.read_word(0x33)
        gy = self.read_word(0x35)
        gz = self.read_word(0x37)

        gx = gx / 65.5
        gy = gy / 65.5
        gz = gz / 65.5

        return gx, gy, gz

    def calibrate_imu(self):
        self.get_logger().info('CALIBRATING IMU - KEEP SENSOR STILL')
        time.sleep(3)

        samples = 500

        for _ in range(samples):
            ax, ay, az = self.read_accel()

            pitch = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
            roll = math.degrees(math.atan2(-ax, az))

            gx, gy, gz = self.read_gyro()

            self.acc_pitch_offset += pitch
            self.acc_roll_offset += roll

            self.gyro_x_offset += gx
            self.gyro_y_offset += gy
            self.gyro_z_offset += gz

            time.sleep(0.005)

        self.acc_pitch_offset /= samples
        self.acc_roll_offset /= samples

        self.gyro_x_offset /= samples
        self.gyro_y_offset /= samples
        self.gyro_z_offset /= samples

        self.get_logger().info(f'Pitch Offset : {self.acc_pitch_offset:.3f} deg')
        self.get_logger().info(f'Roll Offset  : {self.acc_roll_offset:.3f} deg')
        self.get_logger().info(f'Gyro X Offset: {self.gyro_x_offset:.3f} dps')
        self.get_logger().info(f'Gyro Y Offset: {self.gyro_y_offset:.3f} dps')
        self.get_logger().info(f'Gyro Z Offset: {self.gyro_z_offset:.3f} dps')

    def timer_callback(self):
        current_time = time.monotonic()
        dt = min(current_time - self.prev_time, self.MAX_DT_SECONDS)
        self.prev_time = current_time

        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()

        gx -= self.gyro_x_offset
        gy -= self.gyro_y_offset
        gz -= self.gyro_z_offset

        if abs(gz) < self.GYRO_Z_DEADBAND_DPS:
            gz = 0.0

        self.yaw += gz * dt
        self.yaw = math.atan2(math.sin(math.radians(self.yaw)), math.cos(math.radians(self.yaw)))
        yaw_rad = self.yaw

        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)

        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665

        imu_msg.orientation_covariance[0] = 999999.0
        imu_msg.orientation_covariance[4] = 999999.0
        imu_msg.orientation_covariance[8] = 0.05

        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02

        imu_msg.linear_acceleration_covariance[0] = 0.2
        imu_msg.linear_acceleration_covariance[4] = 0.2
        imu_msg.linear_acceleration_covariance[8] = 0.2

        self.imu_pub.publish(imu_msg)

        print(
            f"\rYaw: {math.degrees(yaw_rad):7.2f}° | "
            f"Gyro Z: {gz:7.3f} °/s | "
            f"Accel: x={ax:6.2f} y={ay:6.2f} z={az:6.2f}",
            end=""
        )


def main(args=None):
    rclpy.init(args=args)
    node = ICM20948Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()