import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_common.errors import ErrorCode
from core_interfaces.srv import ParseCommand, Summarize


def unavailable(request, response):
    response.error_code = ErrorCode.NOT_IMPLEMENTED
    if isinstance(response, ParseCommand.Response):
        response.needs_confirmation = True
    return response


def main():
    with ros_context():
        node = Node('llm_node')
        node.create_service(ParseCommand, '/parse_command', unavailable)
        node.create_service(Summarize, '/summarize', unavailable)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
