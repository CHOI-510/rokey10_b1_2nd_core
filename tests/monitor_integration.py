#!/usr/bin/env python3
"""Exercise the Tk monitor against a local, Fake-only Core launch."""

from contextlib import contextmanager
from io import BytesIO
import os
import re
from pathlib import Path
import signal
import subprocess
from tempfile import TemporaryDirectory
from threading import Thread
from time import monotonic
import tkinter as tk

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from PIL import Image as PillowImage
from sensor_msgs.msg import CompressedImage, Image

from core_common import topics
from core_common.ros_context import ros_context
from core_control.monitor_data import MonitorNode
from core_control.monitor_gui import MonitorWindow
from fake_integration import Probe


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise


@contextmanager
def monitor(probe, **parameters):
    parameters.setdefault('preview_enabled', True)
    root = tk.Tk()
    node = MonitorNode(
        cli_args=['--ros-args', '-r', '__node:=monitor_test'],
        parameter_overrides=[Parameter('wrench_topic', value='/monitor_test/wrench'),
                             *(Parameter(name, value=value) for name, value in parameters.items())])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    window = MonitorWindow(root, node)
    threads = [Thread(target=executor.spin), Thread(target=node.decode_images)]
    for thread in threads:
        thread.start()

    def wait(predicate, description, publish=lambda: None):
        deadline = monotonic() + 15
        while True:
            rclpy.spin_once(probe, timeout_sec=0.01)
            publish()
            root.update()
            if predicate():
                return
            assert monotonic() < deadline, description

    try:
        yield root, window, wait
    finally:
        root.destroy()
        node.closed.set()
        node.image_ready.set()
        executor.shutdown()
        for thread in threads:
            thread.join(timeout=5)
            assert not thread.is_alive()
        node.destroy_node()


def exercise(probe):
    image_topic = topics.CAMERA_COLOR
    images = probe.create_publisher(Image, image_topic, 1)
    wrenches = probe.create_publisher(WrenchStamped, '/monitor_test/wrench', 10)
    with monitor(probe) as (root, window, wait):
        def value(key):
            return window.values[key]['text']

        def click(command):
            button = window.buttons[command][0]
            wait(lambda: window.buttons[command][2][0], f'{command} enabled')
            button.event_generate('<ButtonRelease-1>', x=25, y=15)

        def publish_image():
            images.publish(Image(width=2, height=2, step=6, encoding='bgr8',
                                 data=[0, 30, 200] * 4))

        def publish_wrench():
            if probe.status == 'FASTEN':
                sample = WrenchStamped()
                sample.header.frame_id = 'tool'
                sample.header.stamp = probe.get_clock().now().to_msg()
                sample.wrench.force.z = -6.5
                sample.wrench.torque.z = 0.75
                wrenches.publish(sample)

        wait(lambda: value('state') == 'READY', 'GUI received READY')
        assert window.cameras['wrist'].displayed[0] == 'NO SIGNAL'
        assert window.cameras['side'].displayed[0] == 'OFF'
        assert 'N/A' in window.metrics['text']
        wait(lambda: window.cameras['wrist'].displayed[0] == 'LIVE', 'camera live', publish_image)
        assert window.cameras['wrist'].photo is not None
        wait(lambda: window.cameras['wrist'].displayed[0] == 'NO SIGNAL', 'camera timeout')
        wait(lambda: window.cameras['wrist'].displayed[0] == 'LIVE', 'camera recovery', publish_image)
        root.geometry('740x610')
        wait(lambda: window.layout_width is False, 'narrow layout')
        root.geometry('1320x880')
        wait(lambda: window.layout_width is True, 'wide layout')

        def fits(widget, card):
            return (widget.winfo_ismapped() and widget.winfo_height() >= widget.winfo_reqheight()
                    and widget.winfo_rooty() + widget.winfo_height()
                    <= card.body.winfo_rooty() + card.body.winfo_height())

        assert fits(window.values['DB'], window.devices)
        assert fits(window.values['reason'], window.inspection)
        wait(lambda: fits(window.steps['WAIT_WORKER'], window.fsm), 'FSM chips fit in card')

        click('check')
        wait(lambda: value('state') == 'WAIT_WORKER', 'GUI received WAIT_WORKER')
        assert value('top') == 'MISSING'
        assert '현재' in window.steps['WAIT_WORKER']['text']
        click('done')
        wait(lambda: value('final') == 'PASS', 'full path PASS', publish_wrench)
        assert (value('top'), value('side'), value('fastening')) == (
            'PRESENT', 'NOT_PROTRUDING', 'SEATED')
        assert '완료' in window.steps['VERIFY']['text']
        assert 'MOCK' in window.metrics['text'] and '6.50 N' in window.metrics['text']
        journal = window.journal.get('1.0', 'end')
        assert '[INFO]  control / RECHECK' in journal
        assert '0.60 N·m / 1.25' in value('measurement')
        assert probe.results[-1].source == 'gui' and probe.results[-1].tightening_valid
        assert window.journal.yview()[1] == 1.0

        click('check')
        wait(lambda: value('state') == 'WAIT_WORKER', 'second handoff')
        click('cancel')
        wait(lambda: value('reason') == 'cancelled' and value('state') == 'READY', 'CANCEL result')
        assert value('final') == 'HOLD'
        assert 'N/A' in window.metrics['text']
        assert '보류' in window.steps['WAIT_WORKER']['text']

        click('check')
        wait(lambda: value('state') == 'WAIT_WORKER', 'third handoff')
        click('stop')
        stop_started = monotonic()
        wait(lambda: value('state') == 'STOPPED' and value('reason') == 'operator_stop'
             and probe.robot_status.stopped, 'STOP latched in control and Fake Robot', publish_image)
        print(f'GUI STOP both latches during Raw input: {monotonic() - stop_started:.3f}s', flush=True)
        assert not window.buttons['check'][2][0]
        assert not window.buttons['done'][2][0]
        assert '[WARN]' in window.journal.get('1.0', 'end')
        probe.command('check', accepted=False)
        for _ in range(5):
            click('resume')
            wait(lambda: 'resume' not in window.pending, 'RESUME service reply')
            if value('resume') in ('resume_pending', 'confirmed'):
                break
            journal = window.journal.get('1.0', 'end')
            assert 'awaiting_robot_stop_confirmation' in journal
            retry_at = monotonic() + 0.25
            wait(lambda: monotonic() >= retry_at, 'robot stop acknowledgement interval')
        wait(lambda: value('state') == 'READY' and value('resume') == 'confirmed',
             'explicit RESUME confirmed')
        assert len(probe.results) == 3
    compressed = probe.create_publisher(CompressedImage, '/monitor_test/jpeg', 1)
    jpeg = BytesIO()
    PillowImage.new('RGB', (640, 360), (40, 80, 140)).save(jpeg, 'JPEG')
    sample = CompressedImage(format='jpeg', data=jpeg.getvalue())
    with monitor(probe, wrist_image_topic='/monitor_test/jpeg',
                 wrist_image_type='compressed') as (_, window, wait):
        wait(lambda: window.values['state']['text'] == 'READY'
             and window.latest_result == probe.results[-1].task_id, 'restarted monitor replays result')
        assert window.values['final']['text'] == 'HOLD'
        assert window.values['reason']['text'] == 'operator_stop'
        wait(lambda: window.cameras['wrist'].displayed[0] == 'LIVE', 'JPEG preview',
             lambda: compressed.publish(sample))
        assert window.node.frames['wrist'][2] == (640, 360)
    with monitor(probe, preview_enabled=False) as (_, window, wait):
        wait(lambda: window.values['state']['text'] == 'READY', 'control with preview disabled')
        assert window.cameras['wrist'].displayed[0] == 'OFF'
        assert window.buttons['check'][2][0]
        assert not window.node.get_subscriptions_info_by_topic('/monitor_test/jpeg')


def main():
    with ros_context():
        probe = Probe()
        try:
            standalone = subprocess.Popen(
                ['ros2', 'run', 'core_control', 'monitor_gui', '--ros-args',
                 '-r', '__node:=standalone_monitor_test', '-p', 'preview_enabled:=true'], start_new_session=True)
            try:
                probe.wait(lambda: any(endpoint.node_name == 'standalone_monitor_test' for endpoint in
                                       probe.get_subscriptions_info_by_topic(topics.CAMERA_COLOR)),
                           'standalone monitor subscriptions')
                settled_at = monotonic() + 0.4
                probe.wait(lambda: monotonic() >= settled_at, 'standalone GUI event loop')
                assert standalone.poll() is None
            finally:
                stop(standalone)
            probe.wait(lambda: ('standalone_monitor_test', '/') not in probe.get_node_names_and_namespaces(),
                       'standalone monitor shutdown')

            with TemporaryDirectory(prefix='core-monitor-test-') as directory, (
                    Path(directory) / 'launch.log').open('w+') as log:
                launch = subprocess.Popen(
                    ['ros2', 'launch', 'core_bringup', 'system.launch.py', 'mode:=fake', 'gui:=true',
                     f'data_dir:={directory}'], start_new_session=True,
                    stdout=log, stderr=subprocess.STDOUT)
                try:
                    probe.wait(lambda: probe.status == 'READY' and probe.robot_status
                               and probe.robot_status.safe_to_move, 'Fake launch ready')
                    probe.wait(lambda: ('monitor_gui', '/') in probe.get_node_names_and_namespaces(),
                               'launch monitor')
                    exercise(probe)
                    log.seek(0)
                    launched_monitor = re.search(
                        r'\[monitor_gui-\d+\]: process started with pid \[(\d+)\]', log.read())
                    assert launched_monitor
                    os.kill(int(launched_monitor[1]), signal.SIGINT)
                    def monitor_exited():
                        log.seek(0)
                        return (f'process has finished cleanly [pid {launched_monitor[1]}]'
                                in log.read())

                    probe.wait(monitor_exited, 'launched GUI process exited cleanly')
                    probe.command('check')
                    probe.wait(lambda: probe.status == 'WAIT_WORKER', 'control after GUI closure')
                    probe.command('done')
                    probe.wait(lambda: len(probe.results) == 4, 'PASS after GUI closure')
                    assert probe.results[-1].outcome == 'PASS'
                    assert (Path(directory) / 'task_results.jsonl').is_file()
                except Exception:
                    log.seek(0)
                    print(log.read(), flush=True)
                    raise
                finally:
                    stop(launch)
        finally:
            probe.destroy_node()
    print('PASS standalone GUI, Fake launch, five buttons, camera, telemetry and shutdown')


if __name__ == '__main__':
    main()
