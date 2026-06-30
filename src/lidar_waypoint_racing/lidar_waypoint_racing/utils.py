import math

from geometry_msgs.msg import Point


def distance(point_a, point_b):
    """Return the planar Euclidean distance between two (x, y) points."""
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def euler_from_quaternion(quaternion):
    """Convert a geometry_msgs Quaternion to roll, pitch, and yaw."""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)

    return roll, pitch, yaw


def create_point(x, y, z=0.0):
    """Create a geometry_msgs Point with float coordinates."""
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point
