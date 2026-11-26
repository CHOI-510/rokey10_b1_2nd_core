from threading import Event, RLock, Thread
from time import monotonic
from uuid import uuid4

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict

from core_common.ros_context import ros_context
from core_common.config import data_directory
from core_common.errors import ErrorCode as Error
from core_common.reporter import append_jsonl
from core_common.robot_setup import require_fake
from core_common.types import Outcome, State
from core_interfaces.action import Fasten, Fetch, MoveTo, Stow
from core_interfaces.msg import Announcement, ControlStatus, LogEvent, RobotStatus, TaskResult
from core_interfaces.srv import EmergencyStop, Inspect, JobCommand, Locate

from .task_assembly import AssemblyTask, TaskEnded


class Control(Node):
    def __init__(self):
        super().__init__('control')
        require_fake(self.declare_parameter('mode', 'fake').value)
        self.data = data_directory(self.declare_parameter('data_dir', 'data').value)
        self.data.mkdir(parents=True, exist_ok=True)
        self.timeout = self.declare_parameter('request_timeout', 5.0).value
        self.worker_timeout = self.declare_parameter('worker_timeout', 120.0).value
        self.lock = RLock()
        self.shutdown = Event()
        self.state = State.READY
        self.task = None
        self.thread = None
        self.robot_status = None
        self.status_time = 0
        self.stop_pending = None
        self.stop_confirmed = False
        self.resume_pending = None
        self.resume_requested = False
        self.resume_acknowledged = False
        self.actions = {kind: ActionClient(self, kind, '/' + name) for kind, name in (
            (MoveTo, 'move_to'), (Fetch, 'fetch'), (Stow, 'stow'), (Fasten, 'fasten'))}
        self.service_clients = {kind: self.create_client(kind, name) for kind, name in (
            (Inspect, '/inspect'), (Locate, '/locate'), (EmergencyStop, '/robot/emergency_stop'))}
        self.create_service(JobCommand, '/job_command', self.command)
        self.create_service(EmergencyStop, '/emergency_stop', self.emergency_stop)
        self.create_subscription(RobotStatus, '/robot_status', self.receive_status, 10)
        self.status = self.create_publisher(ControlStatus, '/control_status', 10)
        self.logs = self.create_publisher(LogEvent, '/event_log', 10)
        self.results = self.create_publisher(TaskResult, '/task_result', 10)
        self.announcements = self.create_publisher(Announcement, '/announce', 10)
        self.create_timer(0.2, self.tick)

    def stamp(self):
        return self.get_clock().now().to_msg()

    def receive_status(self, msg):
        with self.lock:
            self.robot_status = msg
            self.status_time = monotonic()
            if msg.stopped and self.state != State.STOPPED:
                self.latch_stop('robot_reported_stopped')
            elif self.resume_acknowledged and msg.safe_to_move and not msg.stopped and not msg.busy:
                self.resume_acknowledged = False
                self.stop_confirmed = False
                self.state = State.READY
                self.log('', State.READY, 'operator_resume')
                self.publish_status()

    def publish_status(self):
        self.status.publish(ControlStatus(stamp=self.stamp(), state=self.state,
                                          task_id=self.task.task_id if self.task else ''))

    def journal(self, filename, msg):
        try:
            append_jsonl(self.data / filename, message_to_ordereddict(msg))
        except OSError as exc:
            self.get_logger().error(f'{filename}: {exc}')

    def log(self, task_id, stage, message, level='INFO'):
        msg = LogEvent(stamp=self.stamp(), task_id=task_id, stage=stage,
                       log_level=level, log_msg=message)
        with self.lock:
            self.journal('event_logs.jsonl', msg)
            self.logs.publish(msg)

    def announce(self, task, text):
        self.announcements.publish(Announcement(stamp=self.stamp(), task_id=task.task_id, text=text))

    def transition(self, task, state):
        with self.lock:
            self.guard(task)
            self.state = state
            task.states.append(state)
            self.log(task.task_id, state, 'state_transition')
            self.publish_status()

    def clearance(self):
        return (self.robot_status is not None and monotonic() - self.status_time < 1.0
                and self.robot_status.safe_to_move)

    def require_clearance(self):
        with self.lock:
            if not self.clearance():
                raise TaskEnded(Outcome.HOLD, 'motion_clearance_unavailable')

    def tick(self):
        with self.lock:
            client = self.service_clients[EmergencyStop]
            if (self.state == State.STOPPED and not self.stop_confirmed
                    and not self.stop_pending and not self.resume_pending):
                if client.service_is_ready():
                    self.stop_pending = client.call_async(EmergencyStop.Request(command='stop'))
            if self.stop_pending and self.stop_pending.done():
                self.stop_confirmed = self.stop_pending.result().success
                self.stop_pending = None
            if self.resume_pending and self.resume_pending.done():
                reply = self.resume_pending.result()
                self.resume_pending = None
                if self.resume_requested and reply.success:
                    self.resume_acknowledged = True
                elif self.resume_requested:
                    self.log('', State.STOPPED, reply.reason, 'WARN')
                self.resume_requested = False
            self.publish_status()

    def latch_stop(self, reason):
        with self.lock:
            self.resume_requested = False
            self.resume_acknowledged = False
            self.stop_confirmed = False
            self.state = State.STOPPED
            if self.task and State.STOPPED not in self.task.states:
                self.task.states.append(State.STOPPED)
            self.log(self.task.task_id if self.task else '', State.STOPPED, reason, 'WARN')
            self.publish_status()

    def emergency_stop(self, request, response):
        with self.lock:
            if request.command != 'stop':
                response.error_code = Error.INVALID
                response.reason = 'expected_stop'
                return response
            self.latch_stop('operator_stop')
            self.tick()
            response.success = True
            response.reason = 'control_stop_latched'
            return response

    def command(self, request, response):
        with self.lock:
            command = request.command
            reason = 'command_unavailable_in_state'
            if command == 'check' and self.state == State.READY and self.task is None:
                if self.clearance():
                    self.task = AssemblyTask(self, uuid4().hex)
                    self.thread = Thread(target=self.run_task, args=(self.task,))
                    self.thread.start()
                    response.accepted = True
                else:
                    reason = 'motion_clearance_unavailable'
            elif command == 'done' and self.state == State.WAIT_WORKER and not self.task.done:
                self.task.done = True
                response.accepted = True
            elif command == 'cancel' and self.task and self.state not in (State.REPORT, State.STOPPED):
                self.task.cancelled = True
                response.accepted = True
            elif command == 'resume' and self.state == State.STOPPED and self.task is None:
                client = self.service_clients[EmergencyStop]
                if (self.stop_confirmed and self.resume_pending is None
                        and not self.resume_acknowledged and client.service_is_ready()):
                    self.resume_pending = client.call_async(EmergencyStop.Request(command='resume'))
                    self.resume_requested = True
                    response.accepted = True
                else:
                    reason = 'awaiting_robot_stop_confirmation'
            response.state = self.state
            response.task_id = self.task.task_id if self.task else ''
            response.reason = 'accepted' if response.accepted else reason
            return response

    def guard(self, task):
        with self.lock:
            if self.state == State.STOPPED:
                raise TaskEnded(Outcome.HOLD, 'stopped')
            if self.shutdown.is_set() or task.cancelled:
                raise TaskEnded(Outcome.HOLD, 'cancelled')

    def wait_for(self, task, condition, timeout, reason):
        deadline = monotonic() + timeout
        while True:
            self.guard(task)
            if condition():
                return
            if monotonic() >= deadline:
                raise TaskEnded(Outcome.HOLD, reason)
            self.shutdown.wait(0.02)

    def service(self, task, kind, request):
        client = self.service_clients[kind]
        self.wait_for(task, client.service_is_ready, self.timeout, 'vision_service_unavailable')
        future = client.call_async(request)
        try:
            self.wait_for(task, future.done, self.timeout, 'vision_service_timeout')
            return future.result()
        finally:
            if not future.done():
                future.cancel()

    def action(self, task, kind, goal):
        client = self.actions[kind]
        self.wait_for(task, client.server_is_ready, self.timeout, 'robot_action_unavailable')
        with self.lock:
            self.guard(task)
            self.require_clearance()
            pending = client.send_goal_async(goal, feedback_callback=lambda event: self.log(
                task.task_id, event.feedback.stage, event.feedback.log_msg, event.feedback.log_level))
        handle = None
        result = None
        try:
            self.wait_for(task, pending.done, self.timeout, 'robot_goal_response_timeout')
            handle = pending.result()
            if not handle.accepted:
                raise TaskEnded(Outcome.HOLD, 'robot_goal_rejected')
            result = handle.get_result_async()
            self.wait_for(task, result.done, self.timeout, 'robot_action_timeout')
            reply = result.result()
            if reply.result.error_code == Error.STOPPED:
                self.latch_stop('robot_action_stopped')
                raise TaskEnded(Outcome.HOLD, 'stopped')
            if reply.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
                raise TaskEnded(Outcome.HOLD, 'robot_action_cancelled')
            return reply.result
        except TaskEnded:
            if handle is None:
                # Acceptance is uncertain. Latch stop before any subsequent task.
                self.latch_stop('robot_goal_acceptance_uncertain')
                pending.add_done_callback(self.cancel_late_goal)
            elif handle.accepted and (result is None or not result.done()):
                handle.cancel_goal_async()
                deadline = monotonic() + self.timeout
                while result is not None and not result.done() and monotonic() < deadline:
                    if self.shutdown.wait(0.02):
                        break
                if result is None or not result.done():
                    self.latch_stop('robot_goal_completion_uncertain')
                elif kind is Fasten:
                    task.episode_id = result.result().result.episode_id
            raise

    @staticmethod
    def cancel_late_goal(future):
        goal = future.result()
        if goal is not None and goal.accepted:
            goal.cancel_goal_async()

    def run_task(self, task):
        try:
            outcome, reason = task.run()
        except TaskEnded as exc:
            outcome, reason = exc.outcome, exc.reason
        except Exception as exc:
            self.get_logger().error(f'Task error: {exc}')
            outcome, reason = Outcome.HOLD, 'internal_error'
            self.latch_stop(reason)
        with self.lock:
            if self.state == State.STOPPED:
                outcome, reason = Outcome.HOLD, 'stopped'
            else:
                if task.cancelled:
                    outcome, reason = Outcome.HOLD, 'cancelled'
                self.state = State.REPORT
                task.states.append(State.REPORT)
                self.log(task.task_id, self.state, 'state_transition')
                self.publish_status()
            result = TaskResult(stamp=self.stamp(), task_id=task.task_id, outcome=outcome,
                                reason=reason, fastening=task.fastening, side=task.side,
                                episode_id=task.episode_id, states=task.states)
            self.journal('task_results.jsonl', result)
            self.results.publish(result)
            self.announce(task, f'{outcome} {reason}')
            self.task = None
            if self.state != State.STOPPED:
                self.state = State.READY
            self.publish_status()


def main():
    with ros_context():
        node = Control()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.shutdown.set()
            if node.thread:
                node.thread.join()
            node.destroy_node()
