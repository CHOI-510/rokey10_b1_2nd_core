"""Latest-only ROS Image input for the fastening routine and slot_view.

Subscription QoS follows the publisher's reliability. A BEST_EFFORT subscriber on a
RELIABLE RealSense topic is QoS-compatible, but a raw 1280x720 frame (~2.7 MB) is split
into many transport fragments; losing any one drops the whole frame and best effort never
resends it, so the callback may never fire. A RELIABLE subscription to a RELIABLE publisher
resends missing fragments.
"""

from contextlib import contextmanager
from threading import Condition, Thread
from time import monotonic, sleep

from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

IMAGE_TYPE = 'sensor_msgs/msg/Image'


class RosImageCamera:
    MAX_AGE_S = 1.0
    DISCOVERY_S = 2.0      # time to wait for the publisher before choosing QoS

    def __init__(self, node, topic, check_abort):
        self.node = node
        self.topic = topic
        self.check_abort = check_abort
        self.bridge = CvBridge()
        self.condition = Condition()
        self.closed = False
        self.sample = None
        self.decode_failure = None
        self.last_read = None
        self.group = MutuallyExclusiveCallbackGroup()
        self.reliability = self.publisher_reliability()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=self.reliability, durability=DurabilityPolicy.VOLATILE)
        self.subscription = node.create_subscription(
            Image, topic, self.receive, qos, callback_group=self.group)
        node.get_logger().info(
            f'camera {topic}: subscribe {self.reliability.name} ({self.publishers_text()})')

    # ------------------------------------------------------------ QoS / diagnosis
    def publisher_reliability(self):
        """RELIABLE when every discovered publisher is RELIABLE, otherwise BEST_EFFORT
        (also when none is found yet; BEST_EFFORT matches either kind)."""
        deadline = monotonic() + self.DISCOVERY_S
        while True:
            infos = self.node.get_publishers_info_by_topic(self.topic)
            if infos:
                if all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos):
                    return ReliabilityPolicy.RELIABLE
                return ReliabilityPolicy.BEST_EFFORT
            if monotonic() >= deadline:
                return ReliabilityPolicy.BEST_EFFORT
            self.check_abort()
            sleep(0.05)

    def publishers_text(self):
        infos = self.node.get_publishers_info_by_topic(self.topic)
        if not infos:
            return 'publisher 없음'
        return 'publisher ' + ', '.join(
            f"{i.node_namespace.rstrip('/')}/{i.node_name} {i.qos_profile.reliability.name}"
            for i in infos)

    def no_image_reason(self):
        infos = self.node.get_publishers_info_by_topic(self.topic)
        if not infos:
            images = sorted(name for name, types in self.node.get_topic_names_and_types()
                            if IMAGE_TYPE in types)
            return ('no images received (publisher 없음 — 카메라 노드 실행 · 토픽 이름 · '
                    f"ROS_DOMAIN_ID 확인. 보이는 Image 토픽: {', '.join(images) or '없음'})")
        return (f'no images received ({self.publishers_text()}, 구독 {self.reliability.name} — '
                f'ros2 topic hz {self.topic} 로 발행 확인)')

    # ------------------------------------------------------------ receive
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

    def stale_reason(self):
        stamp, _, _ = self.sample
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        return f'no fresh images acquired after settling (마지막 영상 stamp 나이 {age:+.2f} s)'

    # ------------------------------------------------------------ read
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
                    reason = self.no_image_reason() if self.sample is None else self.stale_reason()
                    raise RuntimeError(f'{self.topic}: {reason}')
                self.condition.wait(min(0.05, deadline - now))

    def read(self, timeout=0.5):
        """Preview: the newest decoded frame not returned before, or None after timeout."""
        deadline = monotonic() + timeout
        with self.condition:
            while True:
                if self.closed:
                    raise RuntimeError(f'{self.topic}: image input closed')
                if (self.sample is not None and self.sample[2] is not None
                        and self.sample[0] != self.last_read):
                    self.last_read = self.sample[0]
                    return self.sample[2]
                left = deadline - monotonic()
                if left <= 0:
                    return None
                self.condition.wait(left)

    def close(self):
        with self.condition:
            if self.closed:
                return
            self.closed = True
            self.sample = self.decode_failure = None
            self.condition.notify_all()
        self.node.destroy_subscription(self.subscription)


@contextmanager
def ros_camera_factory(topic, node_name='fasten_test_camera'):
    """Own node + executor thread for CLI tools whose main node is not spun
    (DSR_ROBOT2 spins it itself). Yields factory(check_abort) -> RosImageCamera."""
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    node = rclpy.create_node(node_name, use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = Thread(target=spin, name=f'{node_name}-spin', daemon=True)
    thread.start()
    try:
        yield lambda check_abort: RosImageCamera(node, topic, check_abort)
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_node()
