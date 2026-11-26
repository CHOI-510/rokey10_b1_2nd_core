import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_common.config import SCENARIOS
from core_common.errors import ErrorCode as Error
from core_common.robot_setup import require_fake
from core_common.types import Presence, Side
from core_common import topics
from core_interfaces.srv import Inspect, Locate

from .inspect import inspect
from .locate import locate


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        require_fake(self.declare_parameter('mode', 'fake').value)
        self.scenario = self.declare_parameter('scenario', 'missing').value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'Unknown scenario {self.scenario}')
        self.create_service(Inspect, topics.INSPECT, self.inspect)
        self.create_service(Locate, topics.LOCATE, self.locate)

    def inspect(self, request, response):
        response.presence, response.side = Presence.UNKNOWN, Side.UNKNOWN
        if request.view_id not in ('top', 'side') or request.stage not in ('INSPECT', 'RECHECK', 'VERIFY'):
            response.error_code = Error.INVALID
            return response
        response.success = True
        response.presence, response.side = inspect(self.scenario, request.stage, request.view_id)
        if response.presence == Presence.PRESENT:
            response.bolt_pose = locate('bolt', self.get_clock().now().to_msg())
        return response

    def locate(self, request, response):
        response.pose = locate(request.item_id, self.get_clock().now().to_msg())
        response.success = response.pose.valid
        response.error_code = Error.OK if response.success else Error.INVALID
        return response


def main():
    with ros_context():
        node = VisionNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
