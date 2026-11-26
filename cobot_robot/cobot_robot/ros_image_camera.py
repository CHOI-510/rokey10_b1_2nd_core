"""Latest-only ROS Image input for the fastening routine."""

from copy import copy
from threading import Condition
from time import monotonic

from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RosImageCamera:
    MAX_AGE_S = 1.0

    def __init__(self, node, topic, check_abort):
        self.node = node
        self.topic = topic
        self.check_abort = check_abort
        self.bridge = CvBridge()
        self.condition = Condition()
        self.closed = False
        self.sample = None
        self.decode_failure = None
        self.group = MutuallyExclusiveCallbackGroup()
        qos = copy(qos_profile_sensor_data)
        qos.depth = 1
        self.subscription = node.create_subscription(
            Image, topic, self.receive, qos, callback_group=self.group)

    def receive(self, message):
        received = monotonic()
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        frame, failure = None, None
        if self.fresh(stamp, received):
            try:
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                if not frame.size:
                    raise ValueError('empty image')
            except Exception as exc:
                failure = str(exc)
        with self.condition:
            if not self.closed:
                self.sample = (stamp, received, frame)
                if failure is not None:
                    self.decode_failure = (stamp, received, failure)
                self.condition.notify_all()

    def fresh(self, stamp, received):
        age = self.node.get_clock().now().nanoseconds - stamp
        return (stamp > 0 and 0 <= age <= self.MAX_AGE_S * 10**9
                and monotonic() - received <= self.MAX_AGE_S)

    def latest(self, settle_s=0.3, timeout=2.0):
        return self.frames(n=1, settle_s=settle_s, timeout=timeout)[0]

    def frames(self, n=10, settle_s=0.3, timeout=3.0):
        # Requiring acquisition after the settling boundary also rejects frames
        # taken during motion and delivered late by DDS.
        settle_until = monotonic() + settle_s
        cutoff = self.node.get_clock().now().nanoseconds + int(settle_s * 10**9)
        deadline = settle_until + timeout
        collected = []
        last_stamp = cutoff
        while True:
            self.check_abort()
            with self.condition:
                if self.closed:
                    raise RuntimeError(f'{self.topic}: image input closed')
                now = monotonic()
                if now >= settle_until:
                    if self.decode_failure is not None:
                        stamp, received, detail = self.decode_failure
                        if received >= settle_until and stamp > cutoff:
                            raise RuntimeError(f'{self.topic}: image decode failed: {detail}')
                    if self.sample is not None:
                        stamp, received, frame = self.sample
                        if (frame is not None and received >= settle_until and stamp > last_stamp
                                and self.fresh(stamp, received)):
                            last_stamp = stamp
                            collected.append(self.sample)
                    collected = [sample for sample in collected if self.fresh(*sample[:2])]
                    if len(collected) >= n or (collected and now >= deadline):
                        return [sample[2] for sample in collected]
                if now >= deadline:
                    reason = ('no images received' if self.sample is None
                              else 'no fresh images acquired after settling')
                    raise RuntimeError(f'{self.topic}: {reason}')
                self.condition.wait(min(0.05, deadline - now))

    def close(self):
        with self.condition:
            if self.closed:
                return
            self.closed = True
            self.sample = self.decode_failure = None
            self.condition.notify_all()
        self.node.destroy_subscription(self.subscription)
