from rclpy.qos import DurabilityPolicy, QoSProfile

SNAPSHOT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def time_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def current_stamp(node, stamp, timeout):
    age = node.get_clock().now().nanoseconds - time_ns(stamp)
    return time_ns(stamp) > 0 and -0.25e9 <= age < timeout * 1e9
