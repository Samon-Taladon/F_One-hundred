import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

from . import utils


class DeadReckoningWaypointRecorder(Node):
    def __init__(self):
        super().__init__('dead_reckoning_waypoint_recorder')

        self.declare_parameter('vesc_state_topic', '/sensors/core')
        self.declare_parameter(
            'servo_topic', '/sensors/servo_position_command'
        )
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        self.declare_parameter('speed_to_erpm_offset', 0.0)
        self.declare_parameter('invert_vesc_speed', False)
        self.declare_parameter('steering_to_servo_gain', -1.2135)
        self.declare_parameter('steering_to_servo_offset', 0.5000)
        self.declare_parameter('steering_deadband', 0.01)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('steering_scale', 2.1)
        self.declare_parameter('steering_offset', 0.0)
        self.declare_parameter('output_file', 'waypoints.csv')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.total_distance = 0.0
        self.total_heading_change = 0.0
        self.last_state_stamp = None
        self.received_state = False
        self.received_servo = False
        self.last_waypoint_position = (0.0, 0.0)
        self.waypoints = [(0.0, 0.0)]

        vesc_state_topic = self.get_parameter('vesc_state_topic').value
        servo_topic = self.get_parameter('servo_topic').value
        self.state_subscription = self.create_subscription(
            VescStateStamped,
            vesc_state_topic,
            self.vesc_state_callback,
            10,
        )
        self.servo_subscription = self.create_subscription(
            Float64,
            servo_topic,
            self.servo_callback,
            10,
        )

        self.get_logger().info(
            f'Recording from VESC state {vesc_state_topic} and servo '
            f'{servo_topic}; origin=(0.0, 0.0)'
        )

    def servo_callback(self, msg):
        servo_gain = float(
            self.get_parameter('steering_to_servo_gain').value
        )
        if servo_gain == 0.0:
            self.get_logger().error('steering_to_servo_gain cannot be zero')
            return

        servo_offset = float(
            self.get_parameter('steering_to_servo_offset').value
        )
        self.steering_angle = (
            ((float(msg.data) - servo_offset) / servo_gain)
            * float(self.get_parameter('steering_scale').value)
            + float(self.get_parameter('steering_offset').value)
        )
        steering_deadband = float(
            self.get_parameter('steering_deadband').value
        )
        if abs(self.steering_angle) < steering_deadband:
            self.steering_angle = 0.0
        if not self.received_servo:
            self.received_servo = True
            self.get_logger().info(
                f'Received first servo value: {msg.data:.4f}; '
                f'steering={self.steering_angle:.4f} rad'
            )

    def vesc_state_callback(self, msg):
        current_stamp = Time.from_msg(msg.header.stamp)
        if self.last_state_stamp is None:
            self.last_state_stamp = current_stamp
            self.received_state = True
            self.get_logger().info(
                f'Received first VESC state: {msg.state.speed:.2f} eRPM'
            )
            return

        dt = (current_stamp - self.last_state_stamp).nanoseconds / 1e9
        self.last_state_stamp = current_stamp
        if dt <= 0.0 or dt > 0.5 or not self.received_servo:
            return

        erpm_gain = float(self.get_parameter('speed_to_erpm_gain').value)
        if erpm_gain == 0.0:
            self.get_logger().error('speed_to_erpm_gain cannot be zero')
            return

        erpm_offset = float(self.get_parameter('speed_to_erpm_offset').value)
        erpm = float(msg.state.speed)
        if self.get_parameter('invert_vesc_speed').value:
            erpm = -erpm
        self.speed = (
            ((erpm - erpm_offset) / erpm_gain)
            * float(self.get_parameter('speed_scale').value)
        )

        min_speed = float(self.get_parameter('min_speed').value)
        if abs(self.speed) < min_speed:
            return

        wheelbase = float(self.get_parameter('wheelbase').value)
        if wheelbase <= 0.0:
            self.get_logger().error('wheelbase must be greater than zero')
            return

        yaw_rate = self.speed * math.tan(self.steering_angle) / wheelbase
        distance_step = abs(self.speed) * dt
        heading_step = yaw_rate * dt
        self.total_distance += distance_step
        self.total_heading_change += heading_step
        midpoint_yaw = self.yaw + 0.5 * yaw_rate * dt
        self.x += self.speed * math.cos(midpoint_yaw) * dt
        self.y += self.speed * math.sin(midpoint_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + yaw_rate * dt),
            math.cos(self.yaw + yaw_rate * dt),
        )

        current_position = (self.x, self.y)
        distance_threshold = float(
            self.get_parameter('distance_threshold').value
        )
        if utils.distance(
                current_position, self.last_waypoint_position
        ) >= distance_threshold:
            self.waypoints.append(current_position)
            self.last_waypoint_position = current_position
            self.get_logger().info(
                f'Recorded waypoint: ({self.x:.4f}, {self.y:.4f}), '
                f'speed={self.speed:.3f}, '
                f'steering={self.steering_angle:.3f}'
            )

    def save_waypoints(self):
        output_file = Path(
            self.get_parameter('output_file').value
        ).expanduser().resolve()
        output_file.parent.mkdir(parents=True, exist_ok=True)

        with output_file.open('w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.waypoints)

        self.get_logger().info(
            f'Saved {len(self.waypoints)} waypoints to {output_file}; '
            f'distance={self.total_distance:.3f} m, '
            f'heading_change={math.degrees(self.total_heading_change):.1f} deg, '
            f'closure_error={utils.distance((self.x, self.y), (0.0, 0.0)):.3f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    recorder = DeadReckoningWaypointRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.save_waypoints()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
