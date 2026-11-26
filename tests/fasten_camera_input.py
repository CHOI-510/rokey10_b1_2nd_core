#!/usr/bin/env python3
"""Raw Image input and launch wiring with a simulated controller only."""
from contextlib import contextmanager
from pathlib import Path
import signal
import subprocess
from tempfile import TemporaryDirectory
from threading import Event, Thread
from time import monotonic, sleep
from unittest.mock import patch

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from core_robot.fastening import fasten_core, slot_vision
from core_robot.fastening.fasten_test import ros_camera_factory
from core_common import topics
from core_common.errors import ErrorCode as Error
from core_interfaces.action import Fasten
from core_interfaces.msg import ControlStatus
from core_robot.real_fasten import RealFasten
from core_robot.robot_node import RobotNode
from real_fasten_contract import SimDR


TOPIC = topics.CAMERA_COLOR


@contextmanager
def spin(*nodes):
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    stopped = Event()

    def work():
        while not stopped.is_set():
            executor.spin_once(timeout_sec=0.05)

    thread = Thread(target=work)
    thread.start()
    try:
        yield
    finally:
        stopped.set()
        thread.join()
        executor.shutdown()


def reply(future, timeout=12):
    deadline = monotonic() + timeout
    wait = Event()
    future.add_done_callback(lambda _: wait.set())
    assert future.done() or wait.wait(max(0, deadline - monotonic())), 'ROS response timeout'
    return future.result()


def act(client):
    goal = Fasten.Goal(task_id='camera-test', point_id='R', recipe_id='finish')
    feedback = []
    handle = reply(client.send_goal_async(
        goal, feedback_callback=lambda event: feedback.append(event.feedback.log_msg)))
    assert handle.accepted
    return handle, feedback


def action_cases(data):
    dr = SimDR()
    original_init = RealFasten.__init__

    def injected(instance, *args, **kwargs):
        assert kwargs['camera_topic'] == TOPIC
        original_init(instance, *args, dr=dr, dsr_available=lambda: True, **kwargs)

    params = dict(mode='real', camera_topic=TOPIC, fasten_server_check_s=0.0, data_dir=data)
    with patch.object(RealFasten, '__init__', injected):
        robot = RobotNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    probe = rclpy.create_node('fasten_camera_probe')
    publisher = probe.create_publisher(Image, TOPIC, qos_profile_sensor_data)
    status = probe.create_publisher(ControlStatus, topics.CONTROL_STATUS, 10)

    def heartbeat():
        status.publish(ControlStatus(stamp=probe.get_clock().now().to_msg(), mode='real',
                                     control_id='camera-test-control', task_id='camera-test', state='FASTEN'))

    probe.create_timer(0.05, heartbeat)
    condition = dict(kind='missing')
    calibration = dict(view_h=fasten_core.VIEW_H, view_offset_tool=list(fasten_core.VIEW_OFFSET_TOOL),
                       sponge_mm=fasten_core.SPONGE_MM, image_size=[2, 2],
                       jac_px_per_mm=[[1, 0], [0, 1]])

    def publish():
        kind = condition['kind']
        if kind == 'missing':
            return
        image = Image(width=2, height=2, encoding='rgb8', step=6, data=bytes([1, 2, 3] * 4))
        if kind == 'invalid':
            image.encoding = 'unsupported_encoding'
        stamp = probe.get_clock().now()
        if kind == 'stale':
            stamp -= Duration(seconds=2)
        image.header.stamp = stamp.to_msg()
        publisher.publish(image)

    probe.create_timer(0.03, publish)
    client = ActionClient(probe, Fasten, topics.FASTEN)
    try:
        with (patch.object(RealFasten, 'services_ready', return_value=True),
              patch.object(slot_vision, 'load_calib', return_value=calibration), spin(robot, probe)):
            assert client.wait_for_server(timeout_sec=5)
            deadline = monotonic() + 5
            while not robot.control_live():
                assert monotonic() < deadline, 'control heartbeat not received'
                sleep(0.01)
            for kind, reason in (('missing', 'no images received'), ('stale', 'no fresh images'),
                                 ('invalid', 'image decode failed')):
                condition['kind'] = kind
                handle, feedback = act(client)
                result = reply(handle.get_result_async()).result
                assert result.error_code == Error.UNAVAILABLE and not dr.motions
                deadline = monotonic() + 1
                while not any(reason in msg for msg in feedback):
                    assert monotonic() < deadline, feedback
                    sleep(0.01)
                assert not any(sub.topic_name == TOPIC for sub in robot.subscriptions)

            condition['kind'] = 'missing'
            handle, _ = act(client)
            reply(handle.cancel_goal_async())
            assert reply(handle.get_result_async()).result.error_code == Error.CANCELLED
            assert not dr.motions
            assert not any(sub.topic_name == TOPIC for sub in robot.subscriptions)
            handle, _ = act(client)
            deadline = monotonic() + 2
            while not any(sub.topic_name == TOPIC for sub in robot.subscriptions):
                assert monotonic() < deadline, 'camera subscription not created'
                sleep(0.01)
            with robot.gate:
                robot.latch_stop('test_stop')
            assert reply(handle.get_result_async()).result.error_code == Error.STOPPED
            assert not dr.motions
            assert not any(sub.topic_name == TOPIC for sub in robot.subscriptions)
            with robot.gate:
                robot.stopped = False

            condition['kind'] = 'valid'

            def simulated_run(fastener):
                assert tuple(fastener.cam.latest(settle_s=0)[0, 0]) == (3, 2, 1)
                fastener.torque_samples = 1
                return fasten_core.RESULT_DONE

            with patch.object(fasten_core.Fastener, 'run', simulated_run):
                handle, _ = act(client)
                assert reply(handle.get_result_async()).result.success
            assert dr.motions
            assert not any(sub.topic_name == TOPIC for sub in robot.subscriptions)
    finally:
        probe.destroy_node()
        robot.journal.close()
        robot.destroy_node()
    print('PASS raw Image Action, BGR conversion, pre-motion errors, cancel/stop and cleanup')


def standalone_cases():
    probe = rclpy.create_node('fasten_camera_cli_probe')
    pub = probe.create_publisher(Image, TOPIC, qos_profile_sensor_data)
    try:
        with ros_camera_factory(TOPIC) as factory:
            cam = factory(lambda: None)
            try:
                deadline = monotonic() + 4
                while not pub.get_subscription_count():
                    assert monotonic() < deadline, 'CLI camera subscription discovery'
                    sleep(0.02)
                old = probe.get_clock().now().to_msg()
                image = Image(width=2, height=2, encoding='bgr8', step=6, data=bytes([7] * 12))
                image.header.stamp = old

                def replay():
                    sleep(0.08)
                    pub.publish(image)    # delivered late, acquired before the call

                thread = Thread(target=replay)
                thread.start()
                try:
                    cam.frames(n=1, settle_s=0.03, timeout=0.25)
                except RuntimeError as exc:
                    assert 'no fresh images' in str(exc), exc
                else:
                    raise AssertionError('pre-motion image was reused')
                finally:
                    thread.join()
            finally:
                cam.close()
    finally:
        probe.destroy_node()

    try:
        fasten_core.Fastener(SimDR(), rclpy.logging.get_logger('missing_camera'), 1,
                             calib_vision=True)
    except fasten_core.Abort as exc:
        assert 'ROS Image subscriber' in str(exc)
    else:
        raise AssertionError('vision started without a ROS subscriber')
    print('PASS CLI executor rejects pre-motion frames and vision requires a ROS subscriber')


def launch_case(data):
    command = ['ros2', 'launch', 'core_bringup', 'system.launch.py',
               'mode:=fake', 'gui:=false', f'camera_topic:={TOPIC}', f'data_dir:={data}']
    with (Path(data) / 'launch.log').open('w+') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        probe = rclpy.create_node('fasten_launch_probe')
        try:
            client = probe.create_client(GetParameters, '/robot_node/get_parameters')
            with spin(probe):
                assert client.wait_for_service(timeout_sec=10)
                response = reply(client.call_async(GetParameters.Request(
                    names=['mode', 'camera_topic'])))
                assert response.values[0].string_value == 'fake'
                assert response.values[1].type == Parameter.Type.NOT_SET.value
                assert not any(item.node_name == 'robot_node' for item in
                               probe.get_subscriptions_info_by_topic(TOPIC))
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            probe.destroy_node()
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
    print('PASS fake launch does not configure or subscribe to the real camera')


if __name__ == '__main__':
    with TemporaryDirectory(prefix='fasten-image-') as directory:
        fasten_core.LOG_DIR = directory
        rclpy.init()
        try:
            action_cases(directory)
            standalone_cases()
            launch_case(directory)
        finally:
            rclpy.try_shutdown()
