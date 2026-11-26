from math import isfinite
from time import sleep

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_common.config import SCENARIOS
from core_common.errors import ErrorCode as Error
from core_common.robot_setup import require_fake
from core_common.types import Presence, Side
from core_common import topics
from core_interfaces.srv import Inspect, InspectPositions, Locate

from .inspect import inspect
from .locate import locate


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        require_fake(self.declare_parameter('mode', 'fake').value)
        self.scenario = self.declare_parameter('scenario', 'missing').value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'Unknown scenario {self.scenario}')
        self.inspection_seconds = self.declare_parameter('inspection_seconds', 0.35).value
        if not isfinite(self.inspection_seconds) or self.inspection_seconds < 0:
            raise ValueError('inspection_seconds must be finite and nonnegative')
        self.create_service(InspectPositions, topics.INSPECT_POSITIONS, self.inspect_positions)
        self.create_service(Inspect, topics.INSPECT, self.inspect)
        self.create_service(Locate, topics.LOCATE, self.locate)

    def inspect_positions(self, request, response):
        stage = request.stage
        if not request.task_id or stage not in ('INSPECT', 'RECHECK', 'VERIFY'):
            response.error_code = Error.INVALID
            response.reason = 'invalid_position_request'
            return response
        sleep(self.inspection_seconds)
        left = Presence.PRESENT
        right = Presence.PRESENT
        if stage == 'INSPECT' and self.scenario == 'missing':
            left = Presence.MISSING
        elif self.scenario == 'vision_unknown':
            left = Presence.UNKNOWN
        elif stage == 'VERIFY' and self.scenario == 'sensor_mismatch':
            right = Presence.MISSING
        response.success = True
        response.error_code = Error.OK
        response.stamp = self.get_clock().now().to_msg()
        response.origin = 'FAKE'
        response.verified = False
        response.presence_l, response.presence_r = left, right
        response.reason = f'simulated_l_r:{self.scenario}:{stage}'
        return response

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
