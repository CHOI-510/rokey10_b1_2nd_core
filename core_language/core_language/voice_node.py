import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_interfaces.msg import Announcement


def main():
    with ros_context():
        node = Node('voice_node')
        node.create_subscription(Announcement, '/announce',
                                 lambda msg: node.get_logger().info(msg.text), 10)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
