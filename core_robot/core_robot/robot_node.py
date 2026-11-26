from math import isclose, isfinite
from threading import Lock
from time import monotonic, sleep

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_common.config import FAKE_FRAME, SCENARIOS, data_directory
from core_common.errors import ErrorCode as Error
from core_common.gripper_api import FakeGripper
from core_common.reporter import feedback
from core_common.robot_setup import require_fake
from core_common.types import Fastening
from core_interfaces.action import Fasten, Fetch, MoveTo, Stow
from core_interfaces.msg import RobotStatus
from core_interfaces.srv import EmergencyStop

from . import fasten, fetch, stow
from .episode_recorder import EpisodeRecorder
from .motion import FakeRobot, VIEWS


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        require_fake(self.declare_parameter('mode', 'fake').value)
        self.scenario = self.declare_parameter('scenario', 'missing').value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'Unknown scenario {self.scenario}')
        self.duration = self.declare_parameter('action_seconds', 0.15).value
        self.declare_parameter('fake_motion_clear', True)
        self.data = data_directory(self.declare_parameter('data_dir', 'data').value)
        self.gate = Lock()
        self.busy = False
        self.stopped = False
        self.robot = FakeRobot(self.get_clock(), self.motion_permission)
        self.gripper = FakeGripper()
        group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.create_timer(0.1, self.publish_status)
        self.create_service(EmergencyStop, '/robot/emergency_stop', self.stop, callback_group=group)
        self.servers = [
            ActionServer(self, kind, name, lambda handle, kind=kind: self.execute(kind, handle),
                         goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=group)
            for kind, name in ((MoveTo, '/move_to'), (Fetch, '/fetch'),
                               (Stow, '/stow'), (Fasten, '/fasten'))
        ]

    def motion_permission(self):
        return self.get_parameter('fake_motion_clear').value and not self.stopped

    def accept(self, goal):
        with self.gate:
            if self.busy or not self.robot.safe_to_move():
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def publish_status(self):
        with self.gate:
            status = RobotStatus(stamp=self.get_clock().now().to_msg(), busy=self.busy,
                                 stopped=self.stopped, safe_to_move=self.robot.safe_to_move())
        self.publisher.publish(status)

    def stop(self, request, response):
        with self.gate:
            if request.command == 'stop':
                self.stopped = True
            elif request.command == 'resume':
                if self.busy or not self.get_parameter('fake_motion_clear').value:
                    response.error_code = Error.BUSY
                    response.reason = 'robot_busy_or_clearance_denied'
                    return response
                self.stopped = False
            else:
                response.error_code = Error.INVALID
                response.reason = 'unknown_stop_command'
                return response
        response.success = True
        response.reason = request.command
        return response

    def execute(self, kind, handle):
        goal = handle.request
        result = kind.Result()
        recorder = None
        if kind is Fasten:
            result.state = Fastening.UNKNOWN
        error = Error.FAILED
        try:
            feedback(kind, handle, kind.__name__.upper(), 'Fake action started')
            if kind is Fasten:
                recorder = EpisodeRecorder(
                    self.data / 'episodes', goal.task_id, self.get_clock().now().nanoseconds,
                    FAKE_FRAME, 'fake')
                result.episode_id = recorder.episode_id
                recorder.sample(stamp_ns=self.get_clock().now().nanoseconds,
                                state='FASTEN', stage='START', command=goal.recipe_id)
            deadline = monotonic() + self.duration
            while True:
                with self.gate:
                    if not self.robot.safe_to_move():
                        error = Error.STOPPED
                        break
                    if handle.is_cancel_requested:
                        error = Error.CANCELLED
                        break
                    if monotonic() >= deadline:
                        error = self.run(kind, goal, result)
                        break
                sleep(0.01)
        except Exception as exc:
            self.get_logger().error(f'{kind.__name__}: {exc}')
        finally:
            with self.gate:
                if self.stopped:
                    error = Error.STOPPED
                if recorder:
                    try:
                        now = self.get_clock().now().nanoseconds
                        recorder.sample(stamp_ns=now, state='FASTEN', stage='END')
                        recorder.finish(now, error == Error.OK, error.name, fastening=result.state,
                                        peak_torque_nm=result.peak_torque, turns=result.turns)
                    except OSError as exc:
                        self.get_logger().error(f'Episode write failed: {exc}')
                        if error == Error.OK:
                            error = Error.FAILED
                if error == Error.OK:
                    handle.succeed()
                elif error == Error.CANCELLED:
                    handle.canceled()
                else:
                    handle.abort()
                self.busy = False
        result.success = error == Error.OK
        result.error_code = int(error)
        return result

    def run(self, kind, goal, result):
        if kind is MoveTo:
            if goal.view_id not in VIEWS:
                return Error.INVALID
            result.current_pose = self.robot.move_to(goal.view_id)
        elif kind is Fetch:
            if not (goal.item_id and valid_pose(goal.pick_pose) and goal.dest_id):
                return Error.INVALID
            if not fetch.run(self.gripper, goal):
                return Error.FAILED
        elif kind is Stow:
            if not (goal.item_id and valid_pose(goal.grasp_pose) and goal.holder_id):
                return Error.INVALID
            if not stow.run(self.gripper, goal):
                return Error.FAILED
        elif kind is Fasten:
            if not (goal.point_id and valid_pose(goal.bolt_pose) and goal.recipe_id in fasten.RECIPES):
                return Error.INVALID
            result.state, result.peak_torque, result.turns = fasten.run(goal, self.scenario)
            if result.state == Fastening.NOT_SEATED:
                return Error.FAILED
        return Error.OK


def valid_pose(target):
    header, pose = target.pose.header, target.pose.pose
    position, rotation = pose.position, pose.orientation
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    return (target.valid and header.frame_id == FAKE_FRAME
            and (header.stamp.sec != 0 or header.stamp.nanosec != 0)
            and all(isfinite(value) for value in (position.x, position.y, position.z, *quaternion))
            and isclose(sum(value * value for value in quaternion), 1.0, abs_tol=1e-6))


def main():
    with ros_context():
        node = RobotNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
