#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import smbus2 as smbus
except ImportError:
    import smbus


class Icm20948:
    def __init__(self, bus_number, address):
        self.bus = smbus.SMBus(bus_number)
        self.address = address

    def select_bank(self, bank):
        self.bus.write_byte_data(self.address, 0x7F, bank << 4)

    def write(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def read_word(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value

    def initialize(self):
        self.select_bank(0)
        self.write(0x06, 0x80)
        time.sleep(0.1)
        self.write(0x06, 0x01)
        self.write(0x07, 0x00)

        self.select_bank(2)
        self.write(0x01, 0x04)
        self.write(0x14, 0x04)
        self.select_bank(0)

    def read_accel_g(self):
        self.select_bank(0)
        return (
            self.read_word(0x2D) / 8192.0,
            self.read_word(0x2F) / 8192.0,
            self.read_word(0x31) / 8192.0,
        )

    def read_gyro_dps(self):
        self.select_bank(0)
        return (
            self.read_word(0x33) / 65.5,
            self.read_word(0x35) / 65.5,
            self.read_word(0x37) / 65.5,
        )


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Icm20948ImuPublisher(Node):
    """Read ICM-20948 over I2C and publish sensor_msgs/Imu on /imu."""

    def __init__(self):
        super().__init__('icm20948_imu_publisher')

        self.declare_parameter('bus_number', 7)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('raw_imu_topic', '/imu/data_raw')
        self.declare_parameter('frame_id', 'imu')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_raw_copy', True)
        self.declare_parameter('complementary_alpha', 0.96)
        self.declare_parameter('calibration_samples', 200)
        self.declare_parameter('gyro_deadband_rad_s', 0.01)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.alpha = float(self.get_parameter('complementary_alpha').value)
        self.gyro_deadband = float(self.get_parameter('gyro_deadband_rad_s').value)
        self.publish_raw_copy = bool(self.get_parameter('publish_raw_copy').value)

        self.imu = Icm20948(
            int(self.get_parameter('bus_number').value),
            int(self.get_parameter('i2c_address').value),
        )
        self.imu.initialize()

        self.gyro_offsets = self.calibrate_gyro()
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.last_time = time.monotonic()

        self.imu_pub = self.create_publisher(
            Imu,
            str(self.get_parameter('imu_topic').value),
            10,
        )
        self.raw_pub = self.create_publisher(
            Imu,
            str(self.get_parameter('raw_imu_topic').value),
            10,
        )

        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_imu)
        self.get_logger().info(
            f'ICM-20948 IMU publisher started on '
            f'{self.get_parameter("imu_topic").value}'
        )

    def calibrate_gyro(self):
        samples = max(int(self.get_parameter('calibration_samples').value), 0)
        if samples == 0:
            return (0.0, 0.0, 0.0)

        self.get_logger().info(
            f'Calibrating gyro with {samples} samples; keep IMU still'
        )
        sums = [0.0, 0.0, 0.0]
        for _ in range(samples):
            gx, gy, gz = self.imu.read_gyro_dps()
            sums[0] += gx
            sums[1] += gy
            sums[2] += gz
            time.sleep(0.005)
        offsets = tuple(value / samples for value in sums)
        self.get_logger().info(
            'Gyro offsets dps: '
            f'x={offsets[0]:.4f}, y={offsets[1]:.4f}, z={offsets[2]:.4f}'
        )
        return offsets

    def publish_imu(self):
        ax_g, ay_g, az_g = self.imu.read_accel_g()
        gx_dps, gy_dps, gz_dps = self.imu.read_gyro_dps()

        gx = math.radians(gx_dps - self.gyro_offsets[0])
        gy = math.radians(gy_dps - self.gyro_offsets[1])
        gz = math.radians(gz_dps - self.gyro_offsets[2])

        if abs(gx) < self.gyro_deadband:
            gx = 0.0
        if abs(gy) < self.gyro_deadband:
            gy = 0.0
        if abs(gz) < self.gyro_deadband:
            gz = 0.0

        now = time.monotonic()
        dt = max(now - self.last_time, 1e-6)
        self.last_time = now

        accel_pitch = math.atan2(ay_g, math.sqrt(ax_g * ax_g + az_g * az_g))
        accel_roll = math.atan2(-ax_g, az_g)
        self.pitch = self.alpha * (self.pitch + gx * dt) + (1.0 - self.alpha) * accel_pitch
        self.roll = self.alpha * (self.roll + gy * dt) + (1.0 - self.alpha) * accel_roll
        self.yaw += gz * dt

        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax_g * 9.80665
        msg.linear_acceleration.y = ay_g * 9.80665
        msg.linear_acceleration.z = az_g * 9.80665

        msg.orientation_covariance[0] = 0.05
        msg.orientation_covariance[4] = 0.05
        msg.orientation_covariance[8] = 0.10
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.20
        msg.linear_acceleration_covariance[4] = 0.20
        msg.linear_acceleration_covariance[8] = 0.20

        self.imu_pub.publish(msg)
        if self.publish_raw_copy:
            self.raw_pub.publish(msg)


def main():
    rclpy.init()
    node = Icm20948ImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
