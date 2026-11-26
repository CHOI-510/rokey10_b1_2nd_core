"""Real camera presence service, requiring the team L/R interface."""
import json
import time
from pathlib import Path
from threading import Condition
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from core_common import topics
from core_common.errors import ErrorCode
from .bolt_rgb import classify, validate, consensus


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Vision1(Node):
    def __init__(self):
        super().__init__('vision1_node')
        from core_interfaces.srv import InspectPositions
        if 'presence_r' not in InspectPositions.Response.get_fields_and_field_types():
            raise RuntimeError('Latest L/R InspectPositions interface required')
        self.declare_parameter('camera_topic', topics.CAMERA_COLOR)
        self.declare_parameter('roi_file', '~/.config/core_vision/vision1.json')
        self.declare_parameter('wait_seconds', 1.0)
        self.declare_parameter('max_age_seconds', 0.5)
        self.declare_parameter('real_camera', False)
        self.cv = Condition()
        self.frames = deque(maxlen=60)
        self.bridge = CvBridge()
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, self.param('camera_topic'), self.receive,
                                 qos_profile_sensor_data, callback_group=self.image_group)
        self.create_service(InspectPositions, topics.INSPECT_POSITIONS, self.inspect,
                            callback_group=self.service_group)

    def param(self, name):
        return self.get_parameter(name).value

    def receive(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            frame = None
        with self.cv:
            self.frames.append((ns(msg.header.stamp), msg.header.stamp, frame))
            self.cv.notify_all()

    def inspect(self, req, res):
        res.success = False
        res.verified = False
        res.origin = 'REAL' if self.param('real_camera') else 'TEST'
        res.presence_l = res.presence_r = 'UNKNOWN'
        def fail(code, reason):
            res.error_code = int(code)
            res.reason = reason
            return res
        if req.stage not in ('INSPECT', 'RECHECK', 'VERIFY'):
            return fail(ErrorCode.INVALID, 'Unsupported stage')
        try:
            config = validate(json.loads(Path(self.param('roi_file')).expanduser().read_text()))
            if config['topic'] != self.param('camera_topic'):
                raise ValueError('Camera topic mismatch')
            wait = float(self.param('wait_seconds'))
            age = float(self.param('max_age_seconds'))
            if not 0 < wait <= 10 or not 0 < age <= 10:
                raise ValueError('Invalid timeout')
        except Exception as exc:
            return fail(ErrorCode.INVALID, str(exc))
        deadline = time.monotonic() + wait
        samples = []
        last = max(0, ns(req.not_before)-1)
        stamp = None
        while rclpy.ok() and time.monotonic() < deadline:
            with self.cv:
                now = self.get_clock().now().nanoseconds
                eligible = [f for f in self.frames if last < f[0] <= now and f[0] >= ns(req.not_before) and now-f[0] <= age*1e9]
                if not eligible:
                    self.cv.wait(timeout=max(0, min(.05, deadline-time.monotonic())))
                    continue
            for t, captured, frame in eligible:
                if t <= last:
                    continue
                last = t
                if frame is None:
                    return fail(ErrorCode.FAILED, 'Image decoding failed')
                try:
                    samples.append(classify(frame, config))
                except Exception as exc:
                    return fail(ErrorCode.FAILED, str(exc))
                stamp = captured
                if len(samples) == 5:
                    if self.get_clock().now().nanoseconds-t > age*1e9:
                        return fail(ErrorCode.TIMEOUT, 'Frames expired')
                    res.presence_l, res.presence_r = consensus(samples)
                    res.stamp = stamp
                    res.success = True
                    res.error_code = int(ErrorCode.OK)
                    res.verified = bool(self.param('real_camera'))
                    res.reason = 'RGB ROI inspection' if res.verified else 'Test input; not verified'
                    return res
        return fail(ErrorCode.TIMEOUT, 'Insufficient fresh frames')


def main():
    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = Vision1()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
