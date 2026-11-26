#!/usr/bin/env python3
"""Bridge local ROS JPEG frames to RTP over UDP (JPEG or H.264) and read them as BGR.

--codec must match on sender and viewers (default: jpeg, the original format).
"""

import argparse
import ipaddress
import socket
import time

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import GLib, Gst, GstApp  # noqa: E402, F401

Gst.init(None)

GROUP = "224.1.1.1"
PORT = 6000
TOPIC = "/camera/color/image_raw/compressed"
CODECS = ("jpeg", "h264")
RTP_CAPS = {
    "jpeg": "application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26",
    "h264": "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96",
}
# Receiver: depayload/decode until the appsink. JPEG stays encoded (decoded by OpenCV);
# H.264 is decoded by GStreamer straight to BGR.
RECEIVE_TAIL = {
    "jpeg": "rtpjpegdepay",
    "h264": "rtph264depay ! avdec_h264 max-threads=1 "
            "! videoconvert ! video/x-raw,format=BGR",
}


def _check_bus(pipeline):
    message = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
    if message is not None:
        error, detail = message.parse_error()
        raise RuntimeError(f"{message.src.get_name()}: {error.message} ({detail})")


def _start(pipeline):
    if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        pipeline.set_state(Gst.State.NULL)
        _check_bus(pipeline)
        raise RuntimeError("GStreamer pipeline could not start")


class MulticastCamera:
    """Read the latest available frame as OpenCV BGR, or None on timeout."""

    def __init__(self, interface, group=GROUP, port=PORT, latency_ms=60, *, unicast=False,
                 codec="jpeg"):
        if codec not in CODECS:
            raise ValueError(f"codec must be one of {CODECS}")
        import cv2
        import numpy as np

        self.cv2 = cv2
        self.np = np
        self.codec = codec
        self.pipeline = Gst.parse_launch(
            f'udpsrc name=udp caps="{RTP_CAPS[codec]}" '
            f"! rtpjitterbuffer latency={int(latency_ms)} drop-on-latency=true "
            f"! {RECEIVE_TAIL[codec]} "
            "! appsink name=frames max-buffers=1 drop=true sync=false"
        )
        source = self.pipeline.get_by_name("udp")
        source.set_property("address", "0.0.0.0" if unicast else group)
        source.set_property("port", port)
        source.set_property("auto-multicast", not unicast)
        if not unicast:
            source.set_property("multicast-iface", interface)
        self.sink = self.pipeline.get_by_name("frames")
        _start(self.pipeline)

    def read(self, timeout=0.5):
        _check_bus(self.pipeline)
        sample = self.sink.try_pull_sample(int(timeout * Gst.SECOND))
        if sample is None:
            _check_bus(self.pipeline)
            return None
        data = sample.get_buffer()
        raw = self.np.frombuffer(data.extract_dup(0, data.get_size()), dtype=self.np.uint8)
        if self.codec == "jpeg":
            return self.cv2.imdecode(raw, self.cv2.IMREAD_COLOR)
        structure = sample.get_caps().get_structure(0)
        width, height = structure.get_value("width"), structure.get_value("height")
        stride = raw.size // height  # rows may be padded
        return raw.reshape(height, stride)[:, : width * 3].reshape(height, width, 3).copy()

    def close(self):
        self.pipeline.set_state(Gst.State.NULL)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def send(options):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    fps = options.fps if options.fps is not None else (15 if options.codec == "h264" else 0)
    if options.codec == "h264":
        if fps <= 0:
            raise ValueError("--fps must be > 0 for h264")
        # Decode the ROS JPEG once, encode H.264 once, send the same packets to every client.
        encode = (
            "jpegparse ! jpegdec ! videoconvert ! video/x-raw,format=I420 "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={options.bitrate} "
            f"key-int-max={fps} bframes=0 vbv-buf-capacity=300 "
            f"{'intra-refresh=true ' if options.intra_refresh else ''}"
            "! rtph264pay pt=96 mtu=1200 config-interval=-1 aggregate-mode=zero-latency"
        )
    else:
        encode = "jpegparse ! rtpjpegpay pt=26 mtu=1200"
    framerate = f",framerate={fps}/1" if fps > 0 else ""
    pipeline = Gst.parse_launch(
        f"appsrc name=frames caps=image/jpeg{framerate} is-live=true format=time "
        "do-timestamp=true max-buffers=1 leaky-type=downstream block=false "
        f"! {encode} "
        "! multiudpsink name=udp ttl-mc=1 sync=false async=false send-duplicates=false"
    )
    min_interval = 0.9 / fps if fps > 0 else 0.0
    last_push = 0.0
    sink = pipeline.get_by_name("udp")
    addresses = options.unicast_to or [options.group]
    destinations = ",".join(f"{address}:{options.port}" for address in addresses)
    sink.set_property("clients", destinations)
    if not options.unicast_to:
        sink.set_property("multicast-iface", options.interface)
    source = pipeline.get_by_name("frames")
    reported_format_error = False
    received = 0

    def forward(message):
        nonlocal reported_format_error, received, last_push
        if not message.data or bytes(message.data[:2]) != b"\xff\xd8":
            if not reported_format_error:
                node.get_logger().warning("Expected JPEG from " + options.topic)
                reported_format_error = True
            return
        now = time.monotonic()
        if now - last_push < min_interval:
            return  # throttle to --fps
        last_push = now
        result = source.push_buffer(Gst.Buffer.new_wrapped(bytes(message.data)))
        received += 1
        if result != Gst.FlowReturn.OK:
            raise RuntimeError(f"GStreamer refused JPEG frame: {result.value_nick}")

    rclpy.init(args=[])
    node = None
    try:
        _start(pipeline)
        node = Node("b1_camera_udp_sender")
        node.create_subscription(
            CompressedImage,
            options.topic,
            forward,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        detail = f"h264 {options.bitrate} kbps" if options.codec == "h264" else "jpeg"
        print(f"{options.topic} -> {destinations} ({detail}, "
              f"{f'{fps} FPS cap' if fps > 0 else 'no FPS cap'})", flush=True)
        started = time.monotonic()
        previous_sent = 0
        previous_payload = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            _check_bus(pipeline)
            now = time.monotonic()
            elapsed = now - started
            if options.stats and elapsed >= 2:
                sent = sink.get_property("bytes-served")
                payload = sink.get_property("bytes-to-serve")
                print(f"{received / elapsed:.1f} sent FPS · "
                      f"{(payload - previous_payload) * 8 / elapsed / 1e6:.3f} Mbps RTP input · "
                      f"{(sent - previous_sent) * 8 / elapsed / 1e6:.3f} Mbps aggregate UDP payload · "
                      f"{len(addresses)} destinations", flush=True)
                received, started = 0, now
                previous_sent, previous_payload = sent, payload
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def view(options):
    received = 0
    started_at = time.monotonic()
    with MulticastCamera(
        options.interface, options.group, options.port, options.latency_ms,
        unicast=options.unicast, codec=options.codec,
    ) as camera:
        try:
            if not options.headless:
                camera.cv2.namedWindow("B-1 UDP RGB", camera.cv2.WINDOW_NORMAL)
            while True:
                frame = camera.read(timeout=0.25)
                if frame is not None:
                    received += 1
                    if not options.headless:
                        camera.cv2.imshow("B-1 UDP RGB", frame)
                if not options.headless and camera.cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    return
                now = time.monotonic()
                if now - started_at >= 2:
                    print(f"{received / (now - started_at):.1f} FPS", flush=True)
                    started_at, received = now, 0
        except KeyboardInterrupt:
            pass
        finally:
            if not options.headless:
                camera.cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    for name in ("send", "view"):
        command = modes.add_parser(name)
        command.add_argument("--interface", help="Required for multicast. See ip -br addr.")
        command.add_argument("--group", default=GROUP)
        command.add_argument("--port", type=int, default=PORT)
        command.add_argument("--codec", choices=CODECS, default="jpeg",
                             help="Must match between sender and viewers.")
        if name == "send":
            command.add_argument("--topic", default=TOPIC)
            command.add_argument("--stats", action="store_true",
                                 help="Print ROS input FPS and GStreamer UDP counters.")
            command.add_argument("--unicast-to", action="append", type=ipaddress.IPv4Address,
                                 help="Receiver IPv4 address. Repeat for multiple receivers.")
            command.add_argument("--fps", type=int, default=None,
                                 help="Max frames sent per second (default: 15 for h264, "
                                      "no cap for jpeg).")
            command.add_argument("--bitrate", type=int, default=1500,
                                 help="H.264 bitrate in kbps (default 1500).")
            command.add_argument("--intra-refresh", action="store_true",
                                 help="H.264: spread keyframes over frames to avoid bursts.")
        else:
            command.add_argument("--latency-ms", type=int, default=60)
            command.add_argument("--headless", action="store_true")
            command.add_argument("--unicast", action="store_true", help="Listen for UDP unicast.")
    options = parser.parse_args()
    try:
        unicast = options.unicast_to if options.mode == "send" else options.unicast
        if not unicast:
            if not options.interface:
                raise ValueError("--interface is required for multicast")
            socket.if_nametoindex(options.interface)
            if not ipaddress.IPv4Address(options.group).is_multicast:
                raise ValueError("--group must be an IPv4 multicast address")
        elif options.mode == "send" and any(
            address.is_multicast or address.is_unspecified or int(address) == 0xffffffff
            for address in options.unicast_to
        ):
            raise ValueError("--unicast-to requires a unicast destination")
        if not 1 <= options.port <= 65535:
            raise ValueError("--port must be between 1 and 65535")
        if options.mode == "send":
            send(options)
        else:
            if not 0 <= options.latency_ms <= 1000:
                raise ValueError("--latency-ms must be between 0 and 1000")
            view(options)
    except (OSError, ValueError, RuntimeError, GLib.Error) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
