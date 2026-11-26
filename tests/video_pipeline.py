#!/usr/bin/env python3
"""Compare existing RTP/JPEG unicast and optional local ROS Preview on loopback."""
from io import BytesIO
from pathlib import Path
import signal
import socket
import subprocess
import sys
from threading import Thread
from time import monotonic, process_time

from PIL import Image
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage

from core_common.b1_camera_udp import MulticastCamera
from core_common.ros_context import ros_context
from core_control.monitor_data import MonitorNode


TOPIC = '/monitor_video_test/jpeg'


def run(publisher_node, publisher, sample, fanout, preview):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as extra:
        extra.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        extra.bind(('127.0.0.2', 0))
        extra.setblocking(False)
        port = extra.getsockname()[1]
        script = Path(__file__).resolve().parents[1] / 'core_common/core_common/b1_camera_udp.py'
        command = [sys.executable, str(script), 'send', '--topic', TOPIC, '--port', str(port),
                   '--unicast-to', '127.0.0.1', '--stats']
        if fanout == 2:
            command += ['--unicast-to', '127.0.0.2']
        monitor = MonitorNode(parameter_overrides=[
            Parameter('preview_enabled', value=preview), Parameter('wrist_image_topic', value=TOPIC),
            Parameter('wrist_image_type', value='compressed'),
        ])
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        threads = [Thread(target=executor.spin), Thread(target=monitor.decode_images)]
        with MulticastCamera(None, port=port, unicast=True, latency_ms=35) as camera:
            sender = subprocess.Popen(command)
            for thread in threads:
                thread.start()
            sent = received = extra_bytes = 0
            preview_timestamps = set()
            try:
                deadline = monotonic() + 8
                while publisher.get_subscription_count() < 1 + int(preview):
                    assert sender.poll() is None and monotonic() < deadline, 'sender discovery timeout'
                    rclpy.spin_once(publisher_node, timeout_sec=0.05)
                started = monotonic()
                cpu_started = process_time()
                while monotonic() - started < 5:
                    if monotonic() >= started + sent / 15:
                        sample.header.stamp = publisher_node.get_clock().now().to_msg()
                        publisher.publish(sample)
                        sent += 1
                    frame = camera.read(timeout=0.01)
                    if frame is not None:
                        assert frame.shape == (360, 640, 3)
                        received += 1
                    while True:
                        try:
                            extra_bytes += len(extra.recv(65536))
                        except BlockingIOError:
                            break
                    _, _, _, _, frames, _ = monitor.snapshot()
                    if 'wrist' in frames:
                        preview_timestamps.add(frames['wrist'][1])
                seconds = monotonic() - started
                cpu = (process_time() - cpu_started) / seconds * 100
                assert received >= sent * 0.85, (sent, received)
                assert (extra_bytes > 0) == (fanout == 2)
                assert bool(preview_timestamps) == preview
                print(f'{fanout} destinations · Preview {"ON" if preview else "OFF"} · '
                      f'ROS {sent / seconds:.1f} FPS · UDP decode {received / seconds:.1f} FPS · '
                      f'Preview decode {len(preview_timestamps) / seconds:.1f} FPS · '
                      f'second destination {extra_bytes * 8 / seconds / 1e6:.3f} Mbps · '
                      f'probe CPU {cpu:.1f}% of one core', flush=True)
            finally:
                sender.send_signal(signal.SIGINT)
                try:
                    sender.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    sender.kill()
                    sender.wait()
                monitor.closed.set()
                monitor.image_ready.set()
                executor.shutdown()
                for thread in threads:
                    thread.join()
                monitor.destroy_node()


if __name__ == '__main__':
    jpeg = BytesIO()
    Image.effect_noise((640, 360), 25).convert('RGB').save(jpeg, 'JPEG', quality=70)
    sample = CompressedImage(format='jpeg', data=jpeg.getvalue())
    with ros_context():
        node = Node('synthetic_jpeg')
        publisher = node.create_publisher(CompressedImage, TOPIC, 1)
        try:
            for fanout, preview in ((1, False), (2, False), (2, True)):
                run(node, publisher, sample, fanout, preview)
        finally:
            node.destroy_node()
