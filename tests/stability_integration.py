#!/usr/bin/env python3
"""Fault injection against the Fake nodes on local DDS."""
from contextlib import contextmanager
import os
import signal
import subprocess
from tempfile import TemporaryDirectory
from threading import Event, Thread
from types import SimpleNamespace
from time import monotonic

from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future

from core_common.errors import ErrorCode
from core_common.ros_context import ros_context
from core_control.control import Control
from core_interfaces.action import Fasten
from core_interfaces.srv import EmergencyStop
from core_robot.robot_node import RobotNode
from core_vision.vision_node import VisionNode

from fake_integration import Probe, read_jsonl


def options(**parameters):
    return {'enable_rosout': False, 'parameter_overrides': [Parameter(name, value=value)
                                    for name, value in parameters.items()]}


@contextmanager
def system(*, control=None, robot=None, vision_online=True, fasten_online=True, vision_delay=None):
    with TemporaryDirectory(prefix='core-stability-') as data:
        executor = MultiThreadedExecutor(num_threads=6)
        probe = Probe()
        robot_node = RobotNode(**options(data_dir=data, scenario='partial', **(robot or {})))
        nodes = [robot_node]
        if vision_online:
            vision = VisionNode()
            vision.scenario = 'partial'
            if vision_delay is not None:
                vision.inspection_seconds = vision_delay
            nodes.append(vision)
        if not fasten_online:
            robot_node.servers.pop().destroy()

        def start_control():
            node = Control(**options(data_dir=data, heartbeat_period=0.05, **(control or {})))
            nodes.append(node)
            executor.add_node(node)
            return node

        for node in nodes:
            executor.add_node(node)
        control_node = start_control() if control is not None else None
        direct = probe.create_client(EmergencyStop, '/robot/emergency_stop')
        stop_spinning = Event()

        def spin():
            while not stop_spinning.is_set():
                executor.spin_once(timeout_sec=0.05)

        thread = Thread(target=spin)
        thread.start()
        try:
            probe.wait(direct.service_is_ready, 'direct Robot STOP')
            if control_node:
                probe.wait(lambda: probe.status == 'READY' and probe.job.service_is_ready(),
                           'Fake READY')
            yield probe, control_node, robot_node, direct, start_control
        finally:
            for node in nodes:
                if isinstance(node, Control):
                    node.shutdown.set()
                    if node.thread:
                        node.thread.join(5)
            stop_spinning.set()
            thread.join(timeout=5)
            assert not thread.is_alive()
            executor.shutdown(timeout_sec=8)
            for node in nodes:
                if isinstance(node, (Control, RobotNode)):
                    node.journal.close()
                node.destroy_node()
            probe.destroy_node()


def action_cases():
    cases = [
        ('slow action', {}, {'action_seconds': 0.7}, 'lr_present_after_fake_fasten'),
        ('execution timeout', {'fasten_timeout': 0.2}, {'action_seconds': 1.0},
         'robot_action_timeout'),
        ('feedback timeout', {'feedback_timeout': 0.2},
         {'action_seconds': 1.0, 'fake_action_progress': False}, 'robot_action_feedback_timeout'),
        ('stalled progress', {'progress_timeout': 0.2},
         {'action_seconds': 1.0, 'fake_action_progress': False, 'fake_repeat_feedback': True},
         'robot_action_progress_timeout'),
    ]
    for label, control, robot, reason in cases:
        with system(control=control, robot=robot) as (probe, *_):
            probe.command('check')
            probe.wait(lambda: probe.results, label)
            assert probe.results[0].reason == reason, probe.results[0]
            probe.wait(lambda: probe.status == 'READY', 'verified terminal status')
            print(f'PASS {label}', flush=True)


def watchdog():
    with system(robot={'action_seconds': 2.0, 'control_heartbeat_timeout': 0.35}) as (
            probe, _, _, _, start_control), TemporaryDirectory(prefix='core-watchdog-') as data:
        control = subprocess.Popen([
            'ros2', 'run', 'core_control', 'control', '--ros-args',
            '-p', 'heartbeat_period:=0.05', '-p', f'data_dir:={data}'],
            start_new_session=True)
        try:
            probe.wait(lambda: probe.status == 'READY' and probe.job.service_is_ready(), 'Control up')
            probe.command('check')
            probe.wait(lambda: probe.robot_status.busy, 'active action')
        finally:
            # Kill the process group so neither ros2 run nor its Control child can publish STOP.
            os.killpg(control.pid, signal.SIGKILL)
            control.wait(timeout=5)
        probe.wait(lambda: probe.robot_status.stop_confirmed, 'watchdog stops active action')
        assert probe.robot_status.stop_reason == 'control_heartbeat_lost'
        start_control()
        probe.wait(lambda: probe.status == 'STOPPED', 'replacement Control sees Robot latch')
        probe.command('check', accepted=False)
        assert probe.robot_status.stopped and probe.robot_status.stop_reason == 'control_heartbeat_lost'
        print('PASS Control process loss and restart preserves Robot latch', flush=True)


def direct_stop():
    with system() as (probe, _, _, direct, start_control):
        for _ in range(2):
            response = direct.call_async(EmergencyStop.Request(
                command='stop', source='gui', stop_id='direct-test'))
            probe.wait(response.done, 'direct STOP reply')
            assert response.result().success
        probe.wait(lambda: probe.robot_status and probe.robot_status.stop_confirmed,
                   'direct STOP physically idle in Fake')
        start_control()
        probe.wait(lambda: probe.status == 'STOPPED' and probe.job.service_is_ready(),
                   'Control starts latched')
        probe.command('check', accepted=False)
        probe.command('resume', accepted=False, source='voice')
        print('PASS direct STOP without Control, duplicates, and voice recovery rejection', flush=True)


def lost_replies():
    with system(control={'stop_response_timeout': 0.2, 'resume_response_timeout': 0.3},
                robot={'action_seconds': 0.7}) as (probe, control, _, _, _):
        send = control.start_robot_request
        lost = {'stop', 'resume'}
        requests = []

        def drop_reply(command, request_id):
            requests.append((command, request_id))
            pending = send(command, request_id)
            if pending and command in lost:
                return Future(), pending[1]
            return pending

        control.start_robot_request = drop_reply
        probe.command('check')
        probe.wait(lambda: probe.robot_status.busy, 'action in progress')
        probe.command('stop')
        probe.wait(lambda: control.recovery == 'stop_confirmed' and probe.results,
                   'STOP confirmed from telemetry despite lost response')
        assert probe.results[0].stop_reason == probe.results[0].reason == 'operator_stop'
        probe.wait(lambda: 'resume' in probe.snapshot.allowed_commands, 'bounded STOP request resolved')
        probe.command('resume')
        probe.wait(lambda: control.recovery == 'stop_confirmed' and len(requests) >= 3
                   and probe.robot_status.stop_confirmed, 'lost RESUME causes a fresh latch')
        assert control.state == 'STOPPED' and control.stop_reason == 'operator_stop'
        probe.command('check', accepted=False)
        lost.clear()
        probe.wait(lambda: 'resume' in probe.snapshot.allowed_commands, 're-latch request resolved')
        probe.command('resume')
        probe.wait(lambda: probe.status == 'READY', 'explicit successful retry')
        assert read_jsonl(control.data / 'task_results.jsonl')[0]['stop_reason'] == 'operator_stop'
        print('PASS lost STOP/RESUME replies, re-latch and preserved reason', flush=True)


def bounded_stop():
    with system(control={'stop_attempts': 2, 'stop_response_timeout': 0.15}) as (
            probe, control, _, direct, _):
        requests = []

        def drop_request(command, request_id):
            requests.append((command, request_id))
            return Future(), monotonic() + 0.15

        control.start_robot_request = drop_request
        probe.command('stop')
        probe.wait(lambda: probe.snapshot.recovery == 'stop_unconfirmed', 'STOP retry limit')
        assert len(requests) == 2 and requests[0] == requests[1]
        probe.command('resume', accepted=False)
        reply = direct.call_async(EmergencyStop.Request(
            command='stop', source='gui', stop_id='replacement-latch'))
        probe.wait(reply.done, 'independent STOP')
        probe.wait(lambda: probe.robot_status.stop_confirmed, 'independent latch')
        stale = direct.call_async(EmergencyStop.Request(
            command='resume', source='control', stop_id=probe.robot_status.stop_id))
        probe.wait(stale.done, 'expired RESUME')
        assert not stale.result().success and probe.robot_status.stopped
        print('PASS bounded STOP retries use one ID and expired RESUME is rejected', flush=True)


def uncertain_action():
    with system(control={'fasten_timeout': 0.2, 'action_settle_timeout': 0.2},
                robot={'action_seconds': 1.0}) as (probe, control, _, _, _):
        client = control.actions[Fasten]
        send = client.send_goal_async
        delayed_result = Future()
        actual_results = []

        def hide_result(*args, **kwargs):
            forwarded = Future()

            def accepted(future):
                handle = future.result()
                actual_results.append(handle.get_result_async())
                forwarded.set_result(SimpleNamespace(
                    accepted=handle.accepted, goal_id=handle.goal_id,
                    get_result_async=lambda: delayed_result, cancel_goal_async=handle.cancel_goal_async))

            send(*args, **kwargs).add_done_callback(accepted)
            return forwarded

        client.send_goal_async = hide_result
        probe.command('check')
        probe.wait(lambda: probe.results and probe.robot_status.stop_confirmed, 'unconfirmed result')
        assert probe.results[0].stop_reason == 'robot_goal_completion_uncertain'
        probe.command('resume', accepted=False)
        probe.wait(lambda: actual_results and actual_results[0].done(), 'late terminal result')
        delayed_result.set_result(actual_results[0].result())
        probe.wait(lambda: 'resume' in probe.snapshot.allowed_commands, 'late result clears uncertainty')
        assert probe.snapshot.stop_reason == 'robot_goal_completion_uncertain'
        probe.command('resume')
        probe.wait(lambda: probe.status == 'READY', 'operator resume after late result')
        print('PASS unconfirmed Action stays STOPPED until delayed terminal result and explicit RESUME',
              flush=True)


def blocking_action():
    with system(control={}) as (probe, _, robot, direct, _):
        running, release = Event(), Event()
        run = robot.run

        def blocked(kind, goal, result):
            running.set()
            if not release.wait(3) or robot.stopped:
                return ErrorCode.STOPPED
            return run(kind, goal, result)

        robot.run = blocked
        try:
            probe.command('check')
            probe.wait(running.is_set, 'blocking Fake operation')
            future = direct.call_async(EmergencyStop.Request(command='stop', source='gui'))
            probe.wait(future.done, 'STOP response during blocking operation', timeout=1)
            assert future.result().success
            probe.wait(lambda: probe.robot_status.stopped, 'STOP requested during blocking operation')
            assert probe.robot_status.busy and not probe.robot_status.stop_confirmed
        finally:
            release.set()
        probe.wait(lambda: probe.robot_status.stop_confirmed, 'STOP confirmed after operation exits')
        print('PASS STOP response independent of blocking Fake operation', flush=True)


def internal_error():
    with system(control={}) as (probe, control, _, _, _):
        def fail(*args, **kwargs):
            raise RuntimeError('injected control exception')

        control.action = fail
        probe.command('check')
        probe.wait(lambda: probe.results and probe.snapshot.state == 'STOPPED', 'internal_error')
        assert probe.results[0].reason == probe.results[0].stop_reason == 'internal_error'
        probe.command('stop')
        assert probe.results[0].stop_reason == probe.snapshot.stop_reason == 'internal_error'
        print('PASS internal_error survives later operator STOP', flush=True)


def held_tool():
    with system(control={}) as (probe, _, robot, _, _):
        run = robot.run

        def fail_fasten(kind, goal, result):
            if kind is Fasten:
                robot.gripper.command(0.0, 20.0)
                robot.holding_item = 'driver'
                return ErrorCode.FAILED
            return run(kind, goal, result)

        robot.run = fail_fasten
        probe.command('check')
        probe.wait(lambda: probe.results and probe.status == 'RECOVER', 'tool still held')
        assert probe.results[0].reason == 'l_fastening_unconfirmed'
        probe.command('check', accepted=False)
        probe.command('resume', accepted=False)
        with robot.gate:
            robot.gripper.command(0.1, 0.0)
            robot.holding_item = ''
        probe.wait(lambda: not probe.robot_status.grip_detected, 'operator clears tool')
        probe.command('resume')
        probe.wait(lambda: probe.status == 'READY', 'manual recovery')
        print('PASS held tool forces RECOVER and prohibits CHECK/RESUME until safe', flush=True)


def absent_peers():
    for vision_online, fasten_online, reason in (
            (False, True, 'vision_service_unavailable'),
            (True, False, 'robot_action_unavailable')):
        with system(control={'inspectpositions_timeout': 0.5, 'goal_timeout': 0.5},
                    vision_online=vision_online, fasten_online=fasten_online) as (probe, *_):
            probe.command('check')
            probe.wait(lambda: probe.results, reason)
            result = probe.results[0]
            assert (result.outcome, result.reason) == ('HOLD', reason), result
            assert (result.fastening_l, result.fastening_r) == ('UNKNOWN', 'UNKNOWN')
            assert not result.episode_id
            print(f'PASS fake cycle waits for {reason}', flush=True)


def delayed_vision():
    with system(control={'inspectpositions_timeout': 0.5}, vision_delay=2.0) as (
            probe, _, robot, _, _):
        probe.command('check')
        probe.wait(lambda: probe.snapshot.vision_status == 'WAITING REPLY', 'active vision request')
        assert not probe.results and not robot.busy
        probe.wait(lambda: probe.results, 'vision response timeout')
        result = probe.results[0]
        assert (result.outcome, result.reason) == ('HOLD', 'vision_service_timeout')
        assert not result.episode_id and not robot.last_completed_goal_id
        print('PASS fake cycle holds when discovered Vision does not reply in time', flush=True)


if __name__ == '__main__':
    with ros_context():
        action_cases()
        absent_peers()
        delayed_vision()
        watchdog()
        direct_stop()
        lost_replies()
        bounded_stop()
        uncertain_action()
        blocking_action()
        internal_error()
        held_tool()
