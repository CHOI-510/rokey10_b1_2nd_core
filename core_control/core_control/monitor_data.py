"""ROS and image processing for the monitor. This module owns no Tk objects."""

from collections import deque
from math import isfinite
from io import BytesIO
from queue import Empty, SimpleQueue
from threading import Event, Lock
from time import monotonic
from uuid import uuid4

from PIL import Image as PillowImage
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from core_common.status_qos import SNAPSHOT_QOS, current_stamp

from core_interfaces.msg import ControlStatus, LogEvent, RobotStatus, TaskResult
from core_interfaces.srv import EmergencyStop, JobCommand


COMMANDS = {
    'check': 'CHECK', 'done': 'WORKER DONE', 'resume': 'RESUME',
    'cancel': 'CANCEL', 'stop': 'STOP',
}
SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def stamp_ns(message):
    return message.stamp.sec * 1_000_000_000 + message.stamp.nanosec


class MonitorNode(Node):
    def __init__(self, **node_options):
        super().__init__('monitor_gui', **node_options)
        self.mode = self.declare_parameter('mode', 'fake').value.upper()
        if self.mode not in ('FAKE', 'REAL'):
            raise ValueError('mode must be fake or real')
        self.cameras = {
            'wrist': self.declare_parameter('wrist_image_topic', '/camera/color/image_raw').value,
            'side': self.declare_parameter('side_image_topic', '').value,
        }
        self.preview_enabled = self.declare_parameter('preview_enabled', True).value
        if not self.preview_enabled:
            self.cameras = dict.fromkeys(self.cameras, '')
        self.image_types = {
            camera: self.declare_parameter(f'{camera}_image_type', 'raw').value
            for camera in self.cameras
        }
        self.preview_fps = self.declare_parameter('preview_fps', 10.0).value
        self.preview_size = (
            self.declare_parameter('preview_width', 640).value,
            self.declare_parameter('preview_height', 480).value,
        )
        if (not isfinite(self.preview_fps) or self.preview_fps <= 0
                or min(self.preview_size) <= 0
                or any(kind not in ('raw', 'compressed') for kind in self.image_types.values())):
            raise ValueError('Invalid preview type, FPS or dimensions')
        self.signal_timeout = self.declare_parameter('camera_timeout', 2.0).value
        if not isfinite(self.signal_timeout) or self.signal_timeout <= 0:
            raise ValueError('camera_timeout must be finite and positive')
        wrench_topic = self.declare_parameter('wrench_topic', '').value

        self.lock = Lock()
        self.events = deque(maxlen=1024)
        self.commands = SimpleQueue()
        self.pending = {}
        self.outstanding = set()
        self.closed = Event()
        self.image_ready = Event()
        self.raw_images = {}
        self.frames = {}
        self.image_arrivals = {key: deque(maxlen=256) for key in self.cameras}
        self.control = self.robot = None
        self.equipment = {}
        self.telemetry = None
        self.fasten_start = None

        self.status_timeout = self.declare_parameter('status_timeout', 1.0).value
        if not isfinite(self.status_timeout) or self.status_timeout <= 0:
            raise ValueError('status_timeout must be finite and positive')
        self.job = self.create_client(JobCommand, '/job_command')
        self.stop = self.create_client(EmergencyStop, '/emergency_stop')
        self.robot_stop = self.create_client(EmergencyStop, '/robot/emergency_stop')
        self.create_subscription(ControlStatus, '/control_status', self.receive_control, SNAPSHOT_QOS)
        self.create_subscription(RobotStatus, '/robot_status', self.receive_robot, 10)
        self.create_subscription(LogEvent, '/event_log', self.receive_log, 100)
        self.create_subscription(TaskResult, '/task_result', self.receive_result, SNAPSHOT_QOS)
        for camera, topic in self.cameras.items():
            if topic:
                self.create_subscription(
                    CompressedImage if self.image_types[camera] == 'compressed' else Image,
                    topic, lambda msg, key=camera: self.receive_image(key, msg), SENSOR_QOS)
        if wrench_topic:
            self.create_subscription(WrenchStamped, wrench_topic, self.receive_wrench, SENSOR_QOS)
        self.create_timer(0.05, self.service_commands)
        self.create_timer(0.75, self.discover_equipment)

    def receive_control(self, message):
        if not current_stamp(self, message.stamp, self.status_timeout):
            return
        with self.lock:
            previous = self.control[0] if self.control else None
            if message.task_id and (not previous or previous.task_id != message.task_id):
                if not self.telemetry or self.telemetry['task'] != message.task_id:
                    self.telemetry = None
                    self.fasten_start = None
            if message.state == 'FASTEN' and self.fasten_start is None:
                self.fasten_start = stamp_ns(message)
            self.control = (message, monotonic())
            if not previous or (previous.task_id, previous.state) != (message.task_id, message.state):
                self.events.append(('state', message))

    def receive_robot(self, message):
        if not current_stamp(self, message.stamp, self.status_timeout):
            return
        with self.lock:
            self.robot = (message, monotonic())

    def receive_log(self, message):
        with self.lock:
            if message.stage == 'FASTEN' and message.log_msg == 'state_transition':
                if self.control and self.control[0].task_id == message.task_id:
                    self.fasten_start = stamp_ns(message)
            self.events.append(('log', message))

    def receive_result(self, message):
        with self.lock:
            self.events.append(('result', message))

    def receive_image(self, camera, message):
        with self.lock:
            now = monotonic()
            self.image_arrivals[camera].append(now)
            self.raw_images[camera] = (message, now)
            self.image_ready.set()

    def decode_images(self):
        formats = {'rgb8': ('RGB', 'RGB'), 'bgr8': ('RGB', 'BGR'),
                   'rgba8': ('RGBA', 'RGBA'), 'bgra8': ('RGBA', 'BGRA'),
                   'mono8': ('L', 'L'), '8uc1': ('L', 'L')}
        while not self.closed.is_set():
            self.image_ready.wait(0.2)
            with self.lock:
                images = self.raw_images
                self.raw_images = {}
                self.image_ready.clear()
            for camera, (message, received) in images.items():
                try:
                    if isinstance(message, CompressedImage):
                        with PillowImage.open(BytesIO(message.data)) as encoded:
                            dimensions = encoded.size
                            encoded.thumbnail(self.preview_size, PillowImage.Resampling.BILINEAR)
                            frame = encoded.convert('RGB')
                    else:
                        mode, raw = formats[message.encoding.lower()]
                        dimensions = (message.width, message.height)
                        frame = PillowImage.frombytes(
                            mode, dimensions, bytes(message.data), 'raw', raw, message.step, 1)
                        frame.thumbnail(self.preview_size, PillowImage.Resampling.BILINEAR)
                except (KeyError, ValueError, OSError, PillowImage.DecompressionBombError):
                    continue
                with self.lock:
                    arrivals = [stamp for stamp in self.image_arrivals[camera]
                                if received - 2.0 < stamp <= received]
                    fps = ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                           if len(arrivals) > 1 and arrivals[-1] > arrivals[0] else 0.0)
                    self.frames[camera] = (frame, received, dimensions, fps)
            self.closed.wait(1 / self.preview_fps)

    def receive_wrench(self, message):
        force, torque = message.wrench.force.z, message.wrench.torque.z
        if not (isfinite(force) and isfinite(torque)):
            return
        with self.lock:
            if not self.control or monotonic() - self.control[1] > self.status_timeout:
                return
            status = self.control[0]
            time_ns = stamp_ns(message.header)
            if (status.state != 'FASTEN' or not status.task_id or self.fasten_start is None
                    or time_ns < self.fasten_start or time_ns == 0):
                return
            if not self.telemetry:
                self.telemetry = {
                    'task': status.task_id, 'samples': deque(maxlen=400),
                    'force': 0.0, 'torque': 0.0, 'first': time_ns, 'last': time_ns,
                }
            data = self.telemetry
            if time_ns < data['last']:
                return
            data['last'] = time_ns
            data['force'] = max(data['force'], abs(force))
            data['torque'] = max(data['torque'], abs(torque))
            data['samples'].append(((time_ns - data['first']) / 1e9, force, torque))

    def discover_equipment(self):
        names = {name for name, _ in self.get_node_names_and_namespaces()}
        equipment = {label: 'NODE ONLINE' if name in names else 'OFF' for label, name in (
            ('Voice', 'voice_node'), ('LLM', 'llm_node'),
            ('DB', 'database_node'))}
        for label in ('Voice', 'LLM'):
            if equipment[label] == 'NODE ONLINE':
                equipment[label] = 'NOT_IMPLEMENTED'
        with self.lock:
            self.equipment = equipment

    def service_commands(self):
        while True:
            try:
                command = self.commands.get_nowait()
            except Empty:
                break
            if command in self.outstanding:
                continue
            self.outstanding.add(command)
            paths = (('control', self.stop), ('robot_node', self.robot_stop)) if command == 'stop' else (
                ('control', self.job),)
            stop_id = uuid4().hex if command == 'stop' else ''
            for target, client in paths:
                key = (command, target)
                if key in self.pending:
                    continue
                if not client.service_is_ready():
                    self.command_result(command, target, False, 'service_unavailable')
                    continue
                request = (EmergencyStop.Request(command='stop', source='gui', stop_id=stop_id) if command == 'stop'
                           else JobCommand.Request(command=command, source='gui'))
                try:
                    self.pending[key] = (client, client.call_async(request), monotonic() + 5)
                except Exception as exc:
                    self.command_result(command, target, False, str(exc))
        for (command, target), (client, future, deadline) in list(self.pending.items()):
            if future.done():
                try:
                    reply = future.result()
                    accepted = reply.success if command == 'stop' else reply.accepted
                    self.command_result(command, target, accepted, reply.reason)
                except Exception as exc:
                    self.command_result(command, target, False, str(exc))
            elif monotonic() >= deadline:
                client.remove_pending_request(future)
                future.cancel()
                self.command_result(command, target, False, 'response_timeout; outcome unknown')
            else:
                continue
            del self.pending[command, target]
        completed = self.outstanding - {command for command, _ in self.pending}
        self.outstanding.difference_update(completed)
        with self.lock:
            self.events.extend(('command_done', command) for command in completed)

    def command_result(self, command, target, accepted, reason):
        with self.lock:
            self.events.append(('command', (command, target, accepted, reason)))

    def snapshot(self):
        with self.lock:
            events = list(self.events)
            self.events.clear()
            telemetry = None
            if self.telemetry:
                telemetry = dict(self.telemetry, samples=tuple(self.telemetry['samples']))
            return (events, self.control, self.robot, self.equipment,
                    self.frames.copy(), telemetry)
