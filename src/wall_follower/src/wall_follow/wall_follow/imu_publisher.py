#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import smbus
import time
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

bus = smbus.SMBus(7)
IMU_ADDR = 0x68

def select_bank(bank):
    bus.write_byte_data(IMU_ADDR, 0x7F, bank << 4)

def read_word_2c(addr):
    high = bus.read_byte_data(IMU_ADDR, addr)
    low = bus.read_byte_data(IMU_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.raw_publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.br = TransformBroadcaster(self)

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.alpha = 0.96
        self.last_time = time.time()

        # IMU init
        select_bank(0)
        bus.write_byte_data(IMU_ADDR, 0x06, 0x80)
        time.sleep(0.1)
        bus.write_byte_data(IMU_ADDR, 0x06, 0x01)
        bus.write_byte_data(IMU_ADDR, 0x07, 0x00)
        select_bank(2)
        bus.write_byte_data(IMU_ADDR, 0x14, 0x04)
        bus.write_byte_data(IMU_ADDR, 0x01, 0x04)
        select_bank(0)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        self.get_logger().info('IMU Publisher started')

    def update(self):
        ax = read_word_2c(0x2D) / 8192.0
        ay = read_word_2c(0x2F) / 8192.0
        az = read_word_2c(0x31) / 8192.0
        gx = read_word_2c(0x33) / 65.5
        gy = read_word_2c(0x35) / 65.5
        gz = read_word_2c(0x37) / 65.5

        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        accel_pitch = math.atan2(ay, math.sqrt(ax*ax + az*az))
        accel_roll  = math.atan2(-ax, az)

        self.pitch = self.alpha*(self.pitch + gx_rad*dt) + (1-self.alpha)*accel_pitch
        self.roll  = self.alpha*(self.roll  + gy_rad*dt) + (1-self.alpha)*accel_roll
        self.yaw  += gz_rad * dt

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.angular_velocity.x = gx_rad
        imu_msg.angular_velocity.y = gy_rad
        imu_msg.angular_velocity.z = gz_rad
        imu_msg.linear_acceleration.x = ax * 9.80665
        imu_msg.linear_acceleration.y = ay * 9.80665
        imu_msg.linear_acceleration.z = az * 9.80665

        self.publisher_.publish(imu_msg)
        self.raw_publisher_.publish(imu_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = imu_msg.header.stamp
        tf_msg.header.frame_id = 'base_link'
        tf_msg.child_frame_id = 'imu'
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.br.sendTransform(tf_msg)

def main():
    rclpy.init()
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
