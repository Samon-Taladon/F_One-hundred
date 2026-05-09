#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import time

class BMI160Driver(Node):
    def __init__(self):
        super().__init__('bmi160_driver')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # ปรับค่าเริ่มต้นเป็นบวกเพื่อให้สอดคล้องกัน
        self.last_acc_z = 9.80665 
        self.alpha = 0.15 
        self.address = 0x69 

        self.bus = None
        for bus_id in [8, 1, 7]:
            try:
                temp_bus = smbus2.SMBus(bus_id)
                chip_id = temp_bus.read_byte_data(self.address, 0x00)
                if chip_id is not None:
                    self.bus = temp_bus
                    self.get_logger().info(f'BMI160 found and connected on I2C Bus {bus_id}')
                    break
            except Exception:
                continue

        if self.bus is None:
            self.get_logger().error('Could not find BMI160 on any I2C bus (tried 1, 7, 8). Check wiring!')
            return

        try:
            self.bus.write_byte_data(self.address, 0x7E, 0x11) # Accel
            time.sleep(0.1)
            self.bus.write_byte_data(self.address, 0x7E, 0x15) # Gyro
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Initialization failed: {e}')
            return

        self.timer = self.create_timer(0.05, self.publish_imu_data)

    def read_raw_data(self, addr):
        try:
            data = self.bus.read_i2c_block_data(self.address, addr, 2)
            value = (data[1] << 8) | data[0]
            if value > 32767: 
                value -= 65536
            return value
        except:
            return None

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        raw_z = self.read_raw_data(0x16)
        if raw_z is not None:
            # --- แก้ไขจุดนี้: เปลี่ยนให้เป็นค่าตามจริง (ปกติจะเป็นบวกเมื่อวางหงาย) ---
            current_acc_z = (raw_z / 16384.0) * 9.80665
            
            # การดักจับ Outlier ยังคงไว้เพื่อความเสถียร
            if abs(current_acc_z - self.last_acc_z) > 5.0:
                final_z = self.last_acc_z
            else:
                final_z = (self.alpha * current_acc_z) + ((1.0 - self.alpha) * self.last_acc_z)
            
            self.last_acc_z = final_z
            msg.linear_acceleration.z = final_z
        else:
            msg.linear_acceleration.z = self.last_acc_z

        # อ่านแกน X, Y ปกติ
        msg.linear_acceleration.x = (self.read_raw_data(0x12) / 16384.0) * 9.80665
        msg.linear_acceleration.y = (self.read_raw_data(0x14) / 16384.0) * 9.80665
        
        # อ่านค่า Gyroscope
        msg.angular_velocity.x = (self.read_raw_data(0x0C) / 16.4) * (3.14159 / 180.0)
        msg.angular_velocity.y = (self.read_raw_data(0x0E) / 16.4) * (3.14159 / 180.0)
        msg.angular_velocity.z = (self.read_raw_data(0x10) / 16.4) * (3.14159 / 180.0)

        msg.orientation_covariance[0] = -1.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BMI160Driver()
    try:
        if node.bus is not None:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
