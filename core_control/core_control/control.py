from math import isfinite
from threading import Event, RLock, Thread
from time import monotonic
from uuid import uuid4

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict

from core_common.announcements import result_text
from core_common.config import data_directory
from core_common.errors import ErrorCode as Error
from core_common.reporter import Journal
from core_common.robot_setup import require_fake
from core_common.ros_context import ros_context
from core_common.status_qos import SNAPSHOT_QOS, current_stamp
from core_common.types import Outcome, State
from core_interfaces.action import Fasten, Fetch, MoveTo, Stow
from core_interfaces.msg import Announcement, ControlStatus, LogEvent, RobotStatus, TaskResult
from core_interfaces.srv import EmergencyStop, Inspect, InspectPositions, JobCommand, Locate, ReadFastening

from .task_assembly import AssemblyTask, InspectionTask, RealAssemblyTask, TaskEnded, position_fields


ACTIONS = {MoveTo: 'move_to', Fetch: 'fetch', Stow: 'stow', Fasten: 'fasten'}
SOURCES = {'gui', 'cli', 'voice'}
TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED}


class Control(Node):
    def __init__(self, **node_options):
        super().__init__('control', **node_options)
        self.mode = self.declare_parameter('mode', 'fake').value
        self.inspection_only = self.mode == 'inspection'
        self.real_cycle = self.mode == 'real'
        self.task_type = {'fake': AssemblyTask, 'inspection': InspectionTask,
                          'real': RealAssemblyTask}.get(self.mode)
        if self.task_type is None:
            require_fake(self.mode)
        self.evidence_max_age = self.declare_parameter('evidence_max_age', 3.0).value
        if not isfinite(self.evidence_max_age) or self.evidence_max_age <= 0:
            raise ValueError('evidence_max_age must be positive')
        self.data = data_directory(self.declare_parameter('data_dir', 'data').value)
        self.data.mkdir(parents=True, exist_ok=True)
        self.limits = {key: self.declare_parameter(key + '_timeout', default).value
                       for key, default in {
                           'goal': 5.0, 'move_to': 15.0, 'fetch': 30.0, 'stow': 30.0,
                           'fasten': 30.0, 'cancel': 2.0, 'action_settle': 3.0,
                           'feedback': 3.0, 'progress': 3.0, 'inspect': 5.0, 'locate': 5.0,
                           'robot_heartbeat': 1.0, 'inspectpositions': 5.0, 'readfastening': 10.0,
                           'stop_response': 1.0, 'resume_response': 2.0, 'worker': 120.0,
                       }.items()}
        if self.real_cycle:
            self.limits['fasten'] = self.declare_parameter('real_fasten_timeout', 180.0).value
        self.stop_limit = self.declare_parameter('stop_attempts', 3).value
        if any(not isfinite(limit) or limit <= 0 for limit in self.limits.values()) or self.stop_limit < 1:
            raise ValueError('Timeouts and stop_attempts must be positive')
        heartbeat_period = self.declare_parameter('heartbeat_period', 0.2).value
        if not isfinite(heartbeat_period) or heartbeat_period <= 0:
            raise ValueError('heartbeat_period must be positive')
        self.control_id = uuid4().hex
        self.vision_status = 'UNVERIFIED'
        self.worker_timeout = self.limits['worker']
        self.lock = RLock()
        self.shutdown = Event()
        self.state = State.READY if self.inspection_only else State.RECOVER
        self.starting = not self.inspection_only
        self.recovery = '' if self.inspection_only else 'awaiting_robot_status'
        self.stop_reason = ''
        self.last_reason = ''
        self.last_result = None
        self.task = None
        self.thread = None
        self.robot_status = None
        self.status_time = 0
        self.stop_pending = self.resume_pending = None
        self.resume_ack = None
        self.stop_attempts = 0
        self.stop_retry_at = 0.0
        self.stop_request_id = ''
        self.stop_observed_id = ''
        self.stop_reassert = False
        self.uncertain_action = False
        if self.inspection_only:
            services = ((InspectPositions, '/integration/inspect_positions'),
                        (ReadFastening, '/integration/read_fastening'))
            self.actions = {}
        else:
            vision = ((InspectPositions, '/integration/inspect_positions'),) if self.real_cycle else (
                (Inspect, '/inspect'), (Locate, '/locate'))
            services = (*vision, (EmergencyStop, '/robot/emergency_stop'))
            actions = {Fasten: ACTIONS[Fasten]} if self.real_cycle else ACTIONS
            self.actions = {kind: ActionClient(self, kind, '/' + name) for kind, name in actions}
            self.create_subscription(RobotStatus, '/robot_status', self.receive_status, 10)
        self.service_clients = {kind: self.create_client(kind, name) for kind, name in services}
        self.create_service(JobCommand, '/job_command', self.command)
        self.create_service(EmergencyStop, '/emergency_stop', self.emergency_stop)
        self.status = self.create_publisher(ControlStatus, '/control_status', SNAPSHOT_QOS)
        self.logs = self.create_publisher(LogEvent, '/event_log', 100)
        self.results = self.create_publisher(TaskResult, '/task_result', SNAPSHOT_QOS)
        self.announcements = self.create_publisher(Announcement, '/announce', 10)
        self.journal = Journal(self.data, self.get_logger().error)
        self.create_timer(heartbeat_period, self.tick)

    def stamp(self):
        return self.get_clock().now().to_msg()

    def receive_status(self, message):
        with self.lock:
            if not current_stamp(self, message.stamp, self.limits['robot_heartbeat']):
                return
            if self.real_cycle and message.robot_mode != 'real+real_fasten':
                return
            previous = self.robot_status
            self.robot_status = message
            self.status_time = monotonic()
            if message.stopped:
                reason = message.stop_reason or 'robot_reported_stopped'
                if self.state != State.STOPPED:
                    self.latch_stop(reason)
                elif reason != self.stop_reason and (not previous
                        or previous.stop_reason != message.stop_reason):
                    self.log(self.task.task_id if self.task else '', self.state, reason, 'WARN',
                             node='robot_node')

    def publish_status(self):
        record = self.task or self.last_result
        self.status.publish(ControlStatus(
            stamp=self.stamp(), mode=self.mode, state=self.state,
            control_id=self.control_id, vision_status=self.vision_status,
            task_id=self.task.task_id if self.task else '',
            last_task_id=self.last_result.task_id if self.last_result is not None else '',
            recovery=self.recovery, stop_reason=self.stop_reason,
            top=record.top if record else '', side=record.side if record else '',
            fastening=record.fastening if record else '', outcome=record.outcome if record else '',
            last_reason=self.last_reason, allowed_commands=self.allowed_commands(),
            states=list(record.states) if record else [], **position_fields(record)))

    def log(self, task_id, stage, message, level='INFO', *, node='control', source=''):
        msg = LogEvent(stamp=self.stamp(), task_id=task_id, stage=stage, node=node,
                       source=source, log_level=level, log_msg=message)
        self.journal.submit('event_logs.jsonl', message_to_ordereddict(msg))
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

    def fresh_robot(self):
        return (self.robot_status is not None
                and monotonic() - self.status_time < self.limits['robot_heartbeat']
                and current_stamp(self, self.robot_status.stamp, self.limits['robot_heartbeat']))

    def clearance(self):
        return self.fresh_robot() and self.robot_status.safe_to_move and not self.robot_status.stopped

    def physical_ready(self):
        if not self.fresh_robot():
            return False
        robot = self.robot_status
        if self.real_cycle:
            return (not robot.busy and not robot.current_action
                    and not robot.force_control_active and not robot.compliance_control_active)
        return (not robot.busy and not robot.current_action and robot.current_pose.valid
                and robot.gripper_valid and isfinite(robot.gripper_width)
                and not robot.grip_detected and not robot.holding_item
                and robot.force_state_known and not robot.force_control_active
                and not robot.compliance_control_active)

    def require_clearance(self):
        with self.lock:
            if not self.clearance():
                raise TaskEnded(Outcome.HOLD, 'motion_clearance_unavailable')

    def allowed_commands(self):
        allowed = []
        if self.task:
            if not self.task.cancelled and self.state not in (State.REPORT, State.STOPPED):
                allowed.append('cancel')
                if self.state == State.WAIT_WORKER and not self.task.done:
                    allowed.append('done')
        elif self.inspection_only:
            if self.state == State.READY:
                allowed.append('check')
            elif self.state == State.STOPPED:
                allowed.append('resume')
        elif self.state == State.READY and self.clearance() and self.physical_ready():
            allowed.append('check')
        elif self.state == State.RECOVER and self.clearance() and self.physical_ready():
            allowed.append('resume')
        elif self.state == State.STOPPED and not (
                self.uncertain_action or self.stop_pending or self.resume_pending or self.resume_ack):
            if (self.physical_ready() and self.robot_status.stopped
                    and self.robot_status.stop_confirmed and self.robot_status.stop_id
                    and self.stop_observed_id == self.robot_status.stop_id
                    and self.service_clients[EmergencyStop].service_is_ready()):
                allowed.append('resume')
        return allowed

    def start_stop_cycle(self, *, reassert=False):
        self.stop_attempts = 0
        self.stop_retry_at = monotonic()
        self.stop_request_id = uuid4().hex
        self.stop_observed_id = ''
        self.recovery = 'stop_pending'
        self.stop_reassert = reassert

    def start_robot_request(self, command, request_id):
        client = self.service_clients[EmergencyStop]
        timeout = self.limits[command + '_response']
        expires = self.get_clock().now() + Duration(seconds=timeout)
        request = EmergencyStop.Request(command=command, source='control', stop_id=request_id,
                                        reason=self.stop_reason, expires_at=expires.to_msg())
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.log('', self.state, f'{command}_request_failed: {exc}', 'WARN')
            return None
        return future, monotonic() + timeout

    def robot_reply(self, command):
        attribute = command + '_pending'
        pending = getattr(self, attribute)
        if pending is None:
            return None
        future, deadline = pending
        expired = monotonic() >= deadline
        if not future.done() and not expired:
            return None
        setattr(self, attribute, None)
        if expired:
            if not future.done():
                self.service_clients[EmergencyStop].remove_pending_request(future)
                future.cancel()
            self.log('', self.state, f'{command}_response_timeout', 'WARN')
            return False
        try:
            response = future.result()
            if response.success:
                return True
            self.log('', self.state, f'{command}_rejected: {response.reason}', 'WARN')
        except Exception as exc:
            self.log('', self.state, f'{command}_response_failed: {exc}', 'WARN')
        return False

    def tick(self):
        with self.lock:
            if not self.inspection_only and self.task and not self.fresh_robot():
                self.latch_stop('robot_heartbeat_lost')
            if self.inspection_only:
                pass
            elif self.state == State.STOPPED:
                self.tick_stop()
            elif self.starting and self.fresh_robot():
                if self.clearance() and self.physical_ready():
                    self.starting = False
                    self.state, self.recovery = State.READY, ''
                elif not self.real_cycle:
                    self.starting = False
                    self.recovery = 'physical_state_unconfirmed'
            elif not self.task and self.state == State.READY:
                if not self.clearance() or not self.physical_ready():
                    self.state = State.RECOVER
                    self.recovery = 'physical_state_unconfirmed'
            vision_services = (InspectPositions,) if self.mode != 'fake' else (Inspect, Locate)
            if not all(self.service_clients[kind].service_is_ready() for kind in vision_services):
                self.vision_status = 'UNAVAILABLE'
            elif self.vision_status == 'UNAVAILABLE':
                self.vision_status = 'UNVERIFIED'
            self.publish_status()

    def tick_stop(self):
        self.robot_reply('stop')
        resume = self.robot_reply('resume')
        if resume is True and self.recovery == 'resume_pending':
            self.resume_ack = monotonic() + self.limits['resume_response']
        elif resume is not None:
            self.start_stop_cycle(reassert=True)
        if self.resume_ack:
            if self.clearance() and self.physical_ready():
                self.complete_recovery()
            elif monotonic() >= self.resume_ack:
                self.resume_ack = None
                self.log('', self.state, 'resume_status_timeout', 'WARN')
                self.start_stop_cycle(reassert=True)
            return
        if self.resume_pending:
            return
        robot = self.robot_status
        if self.fresh_robot() and robot.stopped and robot.stop_confirmed and robot.stop_id:
            # During a re-latch, an old stop snapshot cannot confirm the new stop request.
            if not self.stop_reassert or robot.stop_id == self.stop_request_id:
                self.stop_observed_id = robot.stop_id
                self.recovery = 'stop_confirmed'
                return
        if not self.stop_pending and monotonic() >= self.stop_retry_at:
            if self.stop_attempts >= self.stop_limit:
                self.recovery = 'stop_unconfirmed'
            else:
                self.stop_attempts += 1
                if self.service_clients[EmergencyStop].service_is_ready():
                    self.stop_pending = self.start_robot_request('stop', self.stop_request_id)
                else:
                    self.log('', self.state, 'robot_stop_service_unavailable', 'WARN')
                if self.stop_pending is None:
                    self.stop_retry_at = monotonic() + self.limits['stop_response']

    def complete_recovery(self):
        self.resume_ack = None
        self.starting = False
        self.state = State.READY
        self.recovery = 'confirmed'
        self.stop_reason = ''
        self.log('', self.state, 'operator_resume')

    def latch_stop(self, reason, source=''):
        with self.lock:
            if self.state == State.STOPPED and not (self.resume_pending or self.resume_ack):
                if reason != self.stop_reason:
                    self.log(self.task.task_id if self.task else '', self.state,
                             reason, 'WARN', source=source)
                return
            if self.resume_pending:
                future, _ = self.resume_pending
                self.service_clients[EmergencyStop].remove_pending_request(future)
                future.cancel()
                self.resume_pending = None
            self.resume_ack = None
            if self.inspection_only:
                self.recovery = 'inspection_stop_latched'
            else:
                self.start_stop_cycle(reassert=self.state == State.STOPPED)
            if self.state != State.STOPPED:
                self.starting = False
                self.state = State.STOPPED
                self.stop_reason = reason
                self.last_reason = reason
                if self.task:
                    self.task.states.append(State.STOPPED)
                self.log(self.task.task_id if self.task else '', self.state, reason, 'WARN', source=source)
            self.publish_status()

    def emergency_stop(self, request, response):
        with self.lock:
            if request.command != 'stop':
                response.error_code = Error.INVALID
                response.reason = 'expected_stop'
                return response
            self.latch_stop('operator_stop', request.source)
            if not self.inspection_only:
                if self.recovery == 'stop_unconfirmed' and not self.stop_pending:
                    self.start_stop_cycle(reassert=True)
                self.tick_stop()
            self.log(self.task.task_id if self.task else '', self.state,
                     'stop request accepted', source=request.source)
            response.success = True
            response.reason = self.recovery if self.inspection_only else 'control_stop_latched'
            return response

    def command(self, request, response):
        with self.lock:
            command = request.command
            source = request.source
            active_id = self.task.task_id if self.task else ''
            reason = 'command_unavailable_in_state'
            if source not in SOURCES:
                reason = 'invalid_command_source'
            elif command == 'resume' and source == 'voice':
                reason = 'voice_recovery_forbidden'
            elif command == 'stop':
                reason = 'use_emergency_stop'
            elif command in self.allowed_commands():
                if command == 'check':
                    self.recovery = self.last_reason = ''
                    self.task = self.task_type(self, uuid4().hex, source)
                    active_id = self.task.task_id
                    self.thread = Thread(target=self.run_task, args=(self.task,))
                    self.thread.start()
                elif command == 'done':
                    self.task.done = True
                elif command == 'cancel':
                    self.task.cancelled = True
                elif command == 'resume':
                    if self.state == State.STOPPED and not self.inspection_only:
                        self.resume_pending = self.start_robot_request('resume', self.robot_status.stop_id)
                        if self.resume_pending is None:
                            response.reason = 'resume_request_failed'
                            response.state = self.state
                            return response
                        self.recovery = 'resume_pending'
                    else:
                        self.complete_recovery()
                response.accepted = True
                reason = 'accepted'
            elif command == 'resume' and self.state == State.STOPPED:
                reason = 'awaiting_robot_stop_confirmation'
            self.log(active_id, self.state, f'{command}: {reason}',
                     'INFO' if response.accepted else 'WARN', source=source)
            self.publish_status()
            response.state = self.state
            response.task_id = active_id
            response.reason = reason
            return response

    def guard(self, task):
        with self.lock:
            if self.state == State.STOPPED:
                raise TaskEnded(Outcome.HOLD, self.stop_reason)
            if self.shutdown.is_set() or task.cancelled:
                raise TaskEnded(Outcome.HOLD, 'cancelled')

    def wait_for(self, task, condition, timeout, reason, progress=None):
        deadline = monotonic() + timeout
        while True:
            self.guard(task)
            if condition():
                return
            now = monotonic()
            if now >= deadline:
                raise TaskEnded(Outcome.HOLD, reason)
            if progress is not None:
                if now - progress['received'] >= self.limits['feedback']:
                    raise TaskEnded(Outcome.HOLD, 'robot_action_feedback_timeout')
                if now - progress['advanced'] >= self.limits['progress']:
                    raise TaskEnded(Outcome.HOLD, 'robot_action_progress_timeout')
            self.shutdown.wait(0.02)

    def service(self, task, kind, request):
        client = self.service_clients[kind]
        timeout = self.limits[kind.__name__.lower()]
        is_vision = kind is not ReadFastening
        prefix = 'vision' if is_vision else 'fastening_evidence'
        future = None
        try:
            self.wait_for(task, client.service_is_ready, timeout, prefix + '_service_unavailable')
            future = client.call_async(request)
            self.wait_for(task, future.done, timeout, prefix + '_service_timeout')
            reply = future.result()
            if is_vision:
                with self.lock:
                    success_status = 'EXTERNAL REPLY' if self.mode != 'fake' else 'FAKE OK'
                    self.vision_status = (success_status if reply.success else
                                      'NOT_IMPLEMENTED' if reply.error_code == Error.NOT_IMPLEMENTED
                                      else 'ERROR')
            return reply
        except TaskEnded as exc:
            if exc.reason.startswith('vision_service_'):
                with self.lock:
                    self.vision_status = 'UNAVAILABLE' if future is None else 'TIMEOUT'
            raise
        finally:
            if future is not None and not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def action(self, task, kind, goal):
        client = self.actions[kind]
        self.wait_for(task, client.server_is_ready, self.limits['goal'], 'robot_action_unavailable')
        progress = {'value': -1, 'advanced': monotonic(), 'received': monotonic()}

        def receive_feedback(event):
            update = event.feedback
            progress['received'] = monotonic()
            if update.progress > progress['value']:
                progress.update(value=update.progress, advanced=monotonic())
            if update.log_msg:
                self.log(task.task_id, update.stage, update.log_msg, update.log_level, node='robot_node')

        with self.lock:
            self.guard(task)
            self.require_clearance()
            pending = client.send_goal_async(goal, feedback_callback=receive_feedback)
        handle = result = None
        try:
            self.wait_for(task, pending.done, self.limits['goal'], 'robot_goal_response_timeout')
            handle = pending.result()
            if not handle.accepted:
                raise TaskEnded(Outcome.HOLD, 'robot_goal_rejected')
            task.last_goal_id = bytes(handle.goal_id.uuid).hex()
            result = handle.get_result_async()
            progress['advanced'] = progress['received'] = monotonic()
            self.wait_for(task, result.done, self.limits[ACTIONS[kind]], 'robot_action_timeout', progress)
            reply = result.result()
            if kind is Fasten:
                task.capture_fastening(reply.result)
            if reply.result.error_code == Error.STOPPED:
                with self.lock:
                    self.latch_stop(reply.result.stop_reason or 'robot_action_stopped')
                raise TaskEnded(Outcome.HOLD, self.stop_reason)
            if reply.status not in TERMINAL:
                self.mark_uncertain(None, 'robot_goal_completion_uncertain')
                raise TaskEnded(Outcome.HOLD, self.stop_reason)
            if reply.status == GoalStatus.STATUS_CANCELED:
                raise TaskEnded(Outcome.HOLD, 'robot_action_cancelled')
            if self.real_cycle and reply.result.success and reply.status != GoalStatus.STATUS_SUCCEEDED:
                raise TaskEnded(Outcome.HOLD, 'robot_action_result_inconsistent')
            return reply.result
        except TaskEnded:
            if handle is None:
                self.mark_uncertain(None, 'robot_goal_acceptance_uncertain')
                pending.add_done_callback(self.cancel_late_goal)
            elif handle.accepted:
                if result is None or not result.done():
                    self.settle_action(task, kind, handle, result)
                elif (result.cancelled() or result.exception() is not None
                      or result.result().status not in TERMINAL):
                    self.mark_uncertain(result, 'robot_goal_completion_uncertain')
                elif kind is Fasten:
                    task.capture_fastening(result.result().result)
            raise
        except Exception:
            # A transport exception after submission cannot prove the Action ended.
            self.mark_uncertain(result, 'internal_error')
            raise

    def settle_action(self, task, kind, handle, result):
        try:
            cancel = handle.cancel_goal_async()
            self.wait_response(cancel, self.limits['cancel'])
            if not cancel.done():
                cancel.cancel()
                self.log(task.task_id, self.state, 'robot_cancel_response_timeout', 'WARN')
            elif not cancel.result().goals_canceling:
                self.log(task.task_id, self.state, 'robot_cancel_not_accepted', 'WARN')
            if result is not None and self.wait_response(result, self.limits['action_settle']):
                reply = result.result()
                if reply.status in TERMINAL:
                    if kind is Fasten:
                        task.capture_fastening(reply.result)
                    return
        except Exception as exc:
            self.log(task.task_id, self.state, f'robot_cancel_failed: {exc}', 'WARN')
        self.mark_uncertain(result, 'robot_goal_completion_uncertain')

    def wait_response(self, future, timeout):
        deadline = monotonic() + timeout
        while not future.done() and not self.shutdown.is_set() and monotonic() < deadline:
            self.shutdown.wait(0.02)
        return future.done()

    def mark_uncertain(self, future, reason):
        with self.lock:
            self.uncertain_action = True
            self.latch_stop(reason)
        if future is not None:
            future.add_done_callback(self.confirm_terminal)

    def confirm_terminal(self, future):
        if not future.cancelled() and future.exception() is None and future.result().status in TERMINAL:
            with self.lock:
                self.uncertain_action = False

    def cancel_late_goal(self, future):
        if future.cancelled() or future.exception() is not None:
            return
        goal = future.result()
        if goal.accepted:
            goal.cancel_goal_async()
            goal.get_result_async().add_done_callback(self.confirm_terminal)
        else:
            with self.lock:
                self.uncertain_action = False

    def terminal_snapshot(self, task):
        return (not task.last_goal_id or (self.fresh_robot()
                and self.robot_status.last_completed_goal_id == task.last_goal_id))

    def run_task(self, task):
        try:
            outcome, reason = task.run()
        except TaskEnded as exc:
            outcome, reason = exc.outcome, exc.reason
        except Exception as exc:
            self.get_logger().error(f'Task error: {exc}')
            outcome, reason = Outcome.HOLD, 'internal_error'
            self.latch_stop(reason)
        deadline = monotonic() + self.limits['robot_heartbeat']
        while (self.state != State.STOPPED and not self.terminal_snapshot(task)
               and not self.shutdown.is_set() and monotonic() < deadline):
            self.shutdown.wait(0.02)
        with self.lock:
            if self.state == State.STOPPED:
                outcome, reason = Outcome.HOLD, self.stop_reason
            else:
                if task.cancelled:
                    outcome, reason = Outcome.HOLD, 'cancelled'
                self.state = State.REPORT
                task.states.append(State.REPORT)
                self.log(task.task_id, self.state, 'state_transition')
            task.outcome = outcome
            self.last_reason = reason
            result = TaskResult(
                stamp=self.stamp(), task_id=task.task_id, mode=self.mode, source=task.source,
                outcome=outcome, reason=reason, stop_reason=self.stop_reason, top=task.top,
                fastening=task.fastening, side=task.side, episode_id=task.episode_id,
                tightening_valid=task.tightening_valid, peak_torque=task.peak_torque,
                turns=task.turns, recipe_id=task.recipe_id, states=task.states, **position_fields(task))
            self.journal.submit('task_results.jsonl', message_to_ordereddict(result))
            self.last_result = result
            self.results.publish(result)
            self.announce(task, result_text(outcome, reason))
            self.task = None
            if self.state != State.STOPPED:
                if self.inspection_only or (self.terminal_snapshot(task)
                                            and self.clearance() and self.physical_ready()):
                    self.state = State.READY
                else:
                    self.state = State.RECOVER
                    self.recovery = 'physical_state_unconfirmed'
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
            node.journal.close()
            node.destroy_node()
