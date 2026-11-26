from contextlib import contextmanager

import rclpy


@contextmanager
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.try_shutdown()
