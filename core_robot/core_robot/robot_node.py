from collections import deque
from uuid import uuid4

from math import isclose, isfinite
from threading import Event, Lock, Thread
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
from core_common.reporter import Journal, feedback
from core_common.robot_setup import require_fake
from core_common.status_qos import current_stamp, time_ns
from core_common.types import Fastening, State
from core_interfaces.action import Fasten, Fetch, MoveTo, Stow
from core_interfaces.msg import ControlStatus, RobotStatus
from core_interfaces.srv import EmergencyStop

from . import fasten, fetch, stow
from .episode_recorder import EpisodeRecorder
from .motion import FakeRobot, VIEWS


class RobotNode(Node):
    def __init__(self, **node_options):
        super().__init__('robot_node', **node_options)
        self.mode = self.declare_parameter('mode', 'fake').value
        require_fake(self.mode)
        self.scenario = self.declare_parameter('scenario', 'missing').value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'Unknown scenario {self.scenario}')
        self.duration = self.declare_parameter('action_seconds', 0.15).value
        self.declare_parameter('fake_motion_clear', True)
        self.heartbeat_timeout = self.declare_parameter('control_heartbeat_timeout', 1.0).value
        self.require_heartbeat = self.declare_parameter('require_control_heartbeat', True).value
        self.publish_progress = self.declare_parameter('fake_action_progress', True).value
        self.repeat_feedback = self.declare_parameter('fake_repeat_feedback', False).value
        # /fasten backend: 'fake' (default, P0-a) or 'real' (bench fastening routine on the M0609).
        # Only the Fasten action changes; MoveTo/Fetch/Stow stay Fake in both cases.
        self.fasten_backend = self.declare_parameter('fasten_backend', 'fake').value
        if self.fasten_backend not in ('fake', 'real'):
            raise ValueError(f'fasten_backend must be fake or real, got {self.fasten_backend}')
        point_map = list(self.declare_parameter('fasten_point_map', ['assembly_bolt:1']).value)
        fasten_vision = self.declare_parameter('fasten_vision', True).value
        fasten_camera_topic = self.declare_parameter('fasten_camera_topic', '').value
        stop_service = self.declare_parameter(
            'dsr_stop_service', '/dsr01/dsr_controller2/motion/move_stop').value
        duplicate_wait = self.declare_parameter('fasten_server_check_s', 1.0).value
        if not isfinite(self.heartbeat_timeout) or self.heartbeat_timeout <= 0:
            raise ValueError('control_heartbeat_timeout must be positive')
        self.control_received = 0.0
        self.control_state = self.control_id = ''
        self.control_stamp = 0
        self.retired_controls = deque(maxlen=64)
        self.last_completed_goal_id = ''
        self.stop_id = self.stop_reason = self.current_action = self.holding_item = ''
        self.stop_ids = deque(maxlen=64)
        self.error = Error.OK
        self.data = data_directory(self.declare_parameter('data_dir', 'data').value)
        self.gate = Lock()
        self.busy = False
        self.stopped = False
        self.robot = FakeRobot(self.get_clock(), self.motion_permission)
        self.gripper = FakeGripper()
        self.real_fasten = None
        if self.fasten_backend == 'real':
            from .real_fasten import RealFasten
            self.real_fasten = RealFasten(self, point_map, vision=fasten_vision,
                                          stop_service=stop_service,
                                          camera_topic=fasten_camera_topic)
        self.current_pose = self.robot.move_to('top')
        if self.real_fasten is not None:
            # Fake keeps its original startup (no graph check): in-process tests recreate nodes and
            # the graph cache briefly still lists the previous /fasten server.
            self.require_single_fasten_server(duplicate_wait)
        group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.create_subscription(ControlStatus, '/control_status', self.receive_control, 1)
        self.create_timer(0.1, self.watchdog)
        self.create_service(EmergencyStop, '/robot/emergency_stop', self.stop, callback_group=group)
        self.servers = [
            ActionServer(self, kind, name, lambda handle, kind=kind: self.execute(kind, handle),
                         goal_callback=lambda goal, action=kind: self.accept(action, goal), cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=group)
            for kind, name in ((MoveTo, '/move_to'), (Fetch, '/fetch'),
                               (Stow, '/stow'), (Fasten, '/fasten'))
        ]

        self.journal = Journal(self.data, self.get_logger().error, capacity=64)

    def record_episode(self, episode_id, task_id, started, ended, recipe_id, details):
        recorder = EpisodeRecorder(self.data / 'episodes', task_id, started, FAKE_FRAME,
                                   self.mode, episode_id=episode_id)
        try:
            recorder.sample(stamp_ns=started, state='FASTEN', stage='START', command=recipe_id)
            recorder.sample(stamp_ns=ended, state='FASTEN', stage='END')
        finally:
            recorder.finish(ended, **details)

    def require_single_fasten_server(self, wait):
        """Refuse to start when another node already serves /fasten (e.g. Fake and real together)."""
        deadline = monotonic() + max(0.0, wait)
        while True:
            if self.count_publishers('/fasten/_action/status') > 0:
                raise RuntimeError('/fasten is already served by another node; '
                                   'run exactly one robot_node (fasten_backend fake or real)')
            if monotonic() >= deadline:
                return
            sleep(0.05)

    def motion_permission(self):
        return self.get_parameter('fake_motion_clear').value and not self.stopped

    def control_live(self):
        return (monotonic() - self.control_received < self.heartbeat_timeout
                and self.get_clock().now().nanoseconds - self.control_stamp
                < self.heartbeat_timeout * 1e9)

    def receive_control(self, message):
        if (message.mode == self.mode and message.control_id
                and current_stamp(self, message.stamp, self.heartbeat_timeout)):
            with self.gate:
                stamp = time_ns(message.stamp)
                if stamp <= self.control_stamp or message.control_id in self.retired_controls:
                    return
                if self.control_id and self.control_id != message.control_id:
                    self.retired_controls.append(self.control_id)
                    if self.busy:
                        self.latch_stop('control_instance_changed')
                self.control_id, self.control_stamp = message.control_id, stamp
                self.control_received = monotonic()
                self.control_state = message.state
                if self.busy and message.state == State.STOPPED:
                    self.latch_stop(message.stop_reason or 'control_reported_stopped')

    def accept(self, kind, goal):
        if (kind is Fasten and self.real_fasten is not None
                and self.count_publishers('/fasten/_action/status') > 1):
            self.get_logger().error('real /fasten goal rejected: another /fasten server is in the graph')
            return GoalResponse.REJECT
        with self.gate:
            if (self.busy or not self.robot.safe_to_move()
                    or (self.require_heartbeat and (not self.control_live()
                        or self.control_state in (State.STOPPED, State.RECOVER)))):
                return GoalResponse.REJECT
            self.busy = True
            self.current_action = kind.__name__
            return GoalResponse.ACCEPT

    def watchdog(self):
        with self.gate:
            if self.busy and self.require_heartbeat and not self.control_live():
                self.latch_stop('control_heartbeat_lost')
        self.publish_status()

    def publish_status(self):
        with self.gate:
            width, gripped = self.gripper.status()
            status = RobotStatus(
                stamp=self.get_clock().now().to_msg(),
                robot_mode=self.mode if self.real_fasten is None else f'{self.mode}+real_fasten',
                busy=self.busy, stopped=self.stopped, safe_to_move=self.robot.safe_to_move(),
                error_code=self.error, error_message=self.stop_reason if self.stopped else (
                    self.error.name if self.error else ''),
                gripper_valid=True, gripper_width=width, grip_detected=gripped,
                holding_item=self.holding_item, current_action=self.current_action,
                current_pose=self.current_pose, force_state_known=True,
                force_control_active=bool(self.real_fasten and self.real_fasten.force_active),
                compliance_control_active=bool(self.real_fasten and self.real_fasten.force_active), stop_confirmed=self.stopped and not self.busy,
                stop_reason=self.stop_reason, stop_id=self.stop_id,
                last_completed_goal_id=self.last_completed_goal_id)
            self.publisher.publish(status)

    def latch_stop(self, reason, request_id=''):
        if request_id and request_id in self.stop_ids:
            return
        if not self.stopped:
            self.stop_reason = reason
            self.error = Error.STOPPED
            self.robot.request_stop()
            if self.real_fasten is not None:
                self.real_fasten.request_stop()
        if not self.stopped or request_id:
            self.stop_id = request_id or uuid4().hex
            self.stop_ids.append(self.stop_id)
        self.stopped = True

    def stop(self, request, response):
        with self.gate:
            if request.command == 'stop':
                reason = request.reason if request.source == 'control' else 'operator_stop'
                request_id = request.stop_id or uuid4().hex
                self.latch_stop(reason or 'operator_stop', request_id)
                response.reason = ('robot_stop_latched' if self.stopped
                                   else 'stop_request_already_processed')
            elif request.command == 'resume':
                if (request.source != 'control' or not request.stop_id
                        or request.stop_id != self.stop_id
                        or time_ns(request.expires_at) <= self.get_clock().now().nanoseconds
                        or not self.control_live() or self.control_state != State.STOPPED
                        or not self.stopped):
                    response.error_code = Error.INVALID
                    response.reason = 'invalid_or_expired_recovery'
                    return response
                if (self.busy or not self.get_parameter('fake_motion_clear').value
                        or not self.current_pose.valid or self.gripper.status()[1] or self.holding_item):
                    response.error_code = Error.BUSY
                    response.reason = 'robot_state_not_safe_for_recovery'
                    return response
                self.stopped = False
                self.stop_reason = self.stop_id = ''
                self.error = Error.OK
                response.reason = 'robot_stop_released'
            else:
                response.error_code = Error.INVALID
                response.reason = 'unknown_stop_command'
                return response
            response.success = True
        self.publish_status()
        return response

    def execute(self, kind, handle):
        goal = handle.request
        result = kind.Result()
        episode_started = None
        if kind is Fasten:
            result.state = Fastening.UNKNOWN
        error = Error.FAILED
        detail = ''
        try:
            real = kind is Fasten and self.real_fasten is not None
            feedback(kind, handle, kind.__name__.upper(),
                     'Real fasten started' if real else 'Fake action started')
            if kind is Fasten:
                episode_started = self.get_clock().now().nanoseconds
                result.episode_id = f'EP{uuid4().hex}'
            if real:
                error, detail = self.run_real_fasten(handle, goal, result)
            else:
                started = monotonic()
                deadline = started + self.duration
                last_progress = 0
                while True:
                    with self.gate:
                        if not self.robot.safe_to_move():
                            if not self.stopped:
                                self.latch_stop('motion_clearance_lost')
                            error = Error.STOPPED
                            break
                        if handle.is_cancel_requested:
                            error = Error.CANCELLED
                            break
                    if monotonic() >= deadline:
                        error = self.run(kind, goal, result)
                        break
                    if self.publish_progress and self.duration > 0:
                        step = min(99, int(100 * (monotonic() - started) / self.duration))
                        if step > last_progress:
                            last_progress = step
                            feedback(kind, handle, kind.__name__.upper(), '', progress=step)
                    elif self.repeat_feedback:
                        feedback(kind, handle, kind.__name__.upper(), '')
                    sleep(0.01)
        except Exception as exc:
            self.get_logger().error(f'{kind.__name__}: {exc}')
        finally:
            with self.gate:
                if self.stopped:
                    error = Error.STOPPED
                    result.stop_reason = self.stop_reason
                self.error = error
                if error == Error.OK:
                    handle.succeed()
                elif error == Error.CANCELLED:
                    handle.canceled()
                else:
                    handle.abort()
                self.last_completed_goal_id = bytes(handle.goal_id.uuid).hex()
                self.busy = False
                self.current_action = ''
            if episode_started is not None:
                self.journal.enqueue(
                    result.episode_id, self.record_episode, result.episode_id, goal.task_id,
                    episode_started, self.get_clock().now().nanoseconds, goal.recipe_id,
                    dict(success=error == Error.OK, reason=result.stop_reason or error.name,
                         fastening=result.state, peak_torque_nm=result.peak_torque,
                         turns=result.turns, tightening_valid=result.tightening_valid,
                         stop_reason=result.stop_reason, recipe_id=goal.recipe_id,
                         backend=self.fasten_backend, detail=detail,
                         fasten_log=self.real_fasten.last_log if self.real_fasten else ''))
            self.publish_status()
        result.success = error == Error.OK
        result.error_code = int(error)
        return result

    def run_real_fasten(self, handle, goal, result):
        """Real /fasten: blocking routine with cancel/stop checks and periodic feedback.
        Control expects feedback within 3 s and a rising progress within 3 s, so a ticker
        publishes every second and raises progress by one every two seconds (max 98)."""
        if not (goal.point_id and valid_pose(goal.bolt_pose)):
            return Error.INVALID, 'invalid_point_or_pose'
        started = monotonic()
        stage = {'name': 'FASTEN', 'progress': 0}
        done = Event()

        def publish(message='', level='INFO'):
            feedback(Fasten, handle, stage['name'], message, level=level, progress=stage['progress'])

        def notify(name, message):
            stage['name'] = name
            publish(message)

        def tick():
            while not done.wait(1.0):
                stage['progress'] = max(stage['progress'],
                                        min(98, 1 + int((monotonic() - started) / 2)))
                publish()

        def should_abort():
            with self.gate:
                if self.stopped:
                    return 'stopped'
                if not self.robot.safe_to_move():
                    self.latch_stop('motion_clearance_lost')
                    return 'stopped'
                if handle.is_cancel_requested:
                    return 'cancelled'
            return None

        ticker = Thread(target=tick, name='real-fasten-feedback', daemon=True)
        ticker.start()
        try:
            error, detail = self.real_fasten.execute(goal, should_abort, notify, result)
        finally:
            done.set()
            ticker.join()
        level = 'INFO' if error == Error.OK else 'ERROR'
        publish(f'{error.name}: {detail}', level)
        message = (f'real fasten {goal.point_id}/{goal.recipe_id}: {error.name} ({detail}), '
                   f'turns {result.turns:.2f}, peak {result.peak_torque:.2f} N·m, '
                   f'log {self.real_fasten.last_log}')
        if error == Error.OK:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return error, detail

    def run(self, kind, goal, result):
        if kind is MoveTo:
            if goal.view_id not in VIEWS:
                return Error.INVALID
            result.current_pose = self.current_pose = self.robot.move_to(goal.view_id)
        elif kind is Fetch:
            if not (goal.item_id and valid_pose(goal.pick_pose) and goal.dest_id):
                return Error.INVALID
            success = fetch.run(self.gripper, goal)
            self.holding_item = goal.item_id if self.gripper.status()[1] else ''
            if not success:
                return Error.FAILED
        elif kind is Stow:
            if not (goal.item_id and valid_pose(goal.grasp_pose) and goal.holder_id):
                return Error.INVALID
            success = stow.run(self.gripper, goal)
            self.holding_item = goal.item_id if self.gripper.status()[1] else ''
            if not success:
                return Error.FAILED
        elif kind is Fasten:
            if not (goal.point_id and valid_pose(goal.bolt_pose) and goal.recipe_id in fasten.RECIPES):
                return Error.INVALID
            result.state, result.peak_torque, result.turns = fasten.run(goal, self.scenario)
            result.tightening_valid = True
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
            node.journal.close()
            node.destroy_node()
