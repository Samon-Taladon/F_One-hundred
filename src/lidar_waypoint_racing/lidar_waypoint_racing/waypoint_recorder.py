import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
from collections import deque
from pathlib import Path
from . import utils


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('min_recording_speed', 0.05)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('max_position_covariance', 0.25)
        self.declare_parameter('output_file', 'waypoints.csv')

        odom_topic = self.get_parameter('odom_topic').value
        smoothing_window = max(
            1, int(self.get_parameter('smoothing_window').value)
        )
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        self.waypoints = []
        self.last_waypoint_position = None
        self.frame_id = None
        self.rejected_samples = 0
        self.stationary_samples = 0
        self.position_samples = deque(maxlen=smoothing_window)

        self.get_logger().info(
            f'Recording {odom_topic} every '
            f'{self.get_parameter("distance_threshold").value:.3f} m while '
            f'speed is at least '
            f'{self.get_parameter("min_recording_speed").value:.3f} m/s'
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        current_position = (position.x, position.y)
        covariance = msg.pose.covariance
        max_covariance = self.get_parameter('max_position_covariance').value

        if not all(math.isfinite(value) for value in current_position):
            self.rejected_samples += 1
            return

        # A zero covariance is commonly used when the publisher does not
        # provide uncertainty, so only reject explicit non-zero bad estimates.
        position_covariance = max(covariance[0], covariance[7])
        if (max_covariance > 0.0 and position_covariance > 0.0
                and position_covariance > max_covariance):
            self.rejected_samples += 1
            return

        self.position_samples.append(current_position)
        linear_velocity = msg.twist.twist.linear
        speed = math.hypot(linear_velocity.x, linear_velocity.y)
        min_speed = self.get_parameter('min_recording_speed').value
        if speed < min_speed:
            self.stationary_samples += 1
            return

        current_position = (
            sum(point[0] for point in self.position_samples)
            / len(self.position_samples),
            sum(point[1] for point in self.position_samples)
            / len(self.position_samples),
        )

        if self.frame_id is None:
            self.frame_id = msg.header.frame_id
            self.get_logger().info(
                f'Waypoint coordinate frame: {self.frame_id or "<empty>"}'
            )

        if self.last_waypoint_position is None or \
           utils.distance(current_position, self.last_waypoint_position) >= self.get_parameter('distance_threshold').value:
            if current_position not in self.waypoints:
                self.waypoints.append(current_position)
                self.last_waypoint_position = current_position
                self.get_logger().info(f'Recorded waypoint: {current_position}')

    def save_waypoints(self):
        output_file = Path(
            self.get_parameter('output_file').get_parameter_value().string_value
        ).expanduser().resolve()
        output_file.parent.mkdir(parents=True, exist_ok=True)

        with output_file.open('w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.waypoints)
        self.get_logger().info(
            f'Saved {len(self.waypoints)} waypoints to {output_file}; '
            f'ignored {self.stationary_samples} stationary samples and '
            f'rejected {self.rejected_samples} invalid samples'
        )


def main(args=None):
    rclpy.init(args=args)
    waypoint_recorder = WaypointRecorder()
    try:
        rclpy.spin(waypoint_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_recorder.save_waypoints()
        waypoint_recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
