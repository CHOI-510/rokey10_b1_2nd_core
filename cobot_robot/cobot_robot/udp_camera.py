"""B-1 PC 가 보내는 RTP/JPEG UDP 영상을 OpenCV BGR 프레임으로 받는다.

~/Downloads/b1_camera_udp.py 의 MulticastCamera(수신부)만 옮겨 온 것이다.
송신은 B-1 PC 에서:  python3 b1_camera_udp.py send --unicast-to <노트북 IP>
노트북 확인용:        python3 b1_camera_udp.py view --unicast
(view 와 이 모듈은 같은 포트를 쓰므로 동시에 켜지 않는다.)
"""
import time

import cv2
import gi
import numpy as np

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp  # noqa: E402, F401

Gst.init(None)

GROUP = "224.1.1.1"
PORT = 6000


def _check_bus(pipeline):
    message = pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
    if message is not None:
        error, detail = message.parse_error()
        raise RuntimeError(f"{message.src.get_name()}: {error.message} ({detail})")


def port_owners(port=PORT):
    """이 UDP 포트를 이미 잡고 있는 프로세스 이름들 (ss 로 확인, 실패하면 빈 목록)."""
    import re
    import subprocess
    try:
        out = subprocess.run(["ss", "-ulpn"], capture_output=True, text=True, timeout=2).stdout
    except Exception:   # noqa: BLE001
        return []
    owners = []
    for line in out.splitlines():
        if re.search(rf":{port}\b", line):
            owners += re.findall(r'\("([^"]+)",pid=(\d+)', line)
    return [f"{name}(pid {pid})" for name, pid in owners]


class UdpCamera:
    def __init__(self, port=PORT, latency_ms=60, unicast=True, interface=None, group=GROUP):
        # unicast 패킷은 같은 포트를 잡은 소켓 중 하나에만 간다 → 다른 프로세스가 있으면 영상이 그쪽으로 샌다
        # (11/26 17:53: Ctrl+Z 로 멈춘 이전 fasten_test 2 개가 포트를 잡고 있어 3 초 무영상으로 중단)
        busy = port_owners(port)
        if busy:
            raise RuntimeError(
                f"UDP {port} 포트를 이미 사용 중: {', '.join(busy)} — 이전 실행/뷰어를 끄세요 "
                f"(Ctrl+Z 로 멈춘 것이면 그 터미널에서 'kill %1' 또는 'kill -9 <pid>')")
        self.pipeline = Gst.parse_launch(
            'udpsrc name=udp caps="application/x-rtp,media=video,'
            'clock-rate=90000,encoding-name=JPEG,payload=26" '
            f"! rtpjitterbuffer latency={int(latency_ms)} drop-on-latency=true "
            "! rtpjpegdepay ! appsink name=frames max-buffers=1 drop=true sync=false"
        )
        source = self.pipeline.get_by_name("udp")
        source.set_property("address", "0.0.0.0" if unicast else group)
        source.set_property("port", port)
        source.set_property("auto-multicast", not unicast)
        if not unicast and interface:
            source.set_property("multicast-iface", interface)
        self.sink = self.pipeline.get_by_name("frames")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            _check_bus(self.pipeline)
            raise RuntimeError("GStreamer pipeline could not start")

    def read(self, timeout=0.5):
        _check_bus(self.pipeline)
        sample = self.sink.try_pull_sample(int(timeout * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        data = np.frombuffer(buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def latest(self, settle_s=0.3, timeout=2.0):
        """로봇이 멈춘 뒤의 프레임을 얻으려고 settle_s 동안 버리고 마지막 프레임을 돌려준다."""
        frame = None
        t_settle = time.time() + settle_s
        t_end = time.time() + timeout
        while time.time() < t_end:
            f = self.read(timeout=0.2)
            if f is not None:
                frame = f
                if time.time() >= t_settle:
                    return frame
        return frame

    def frames(self, n=10, settle_s=0.3, timeout=3.0):
        """로봇이 멈춘 뒤 settle_s 기다렸다가 서로 다른 프레임 n 장을 모은다."""
        t_settle = time.time() + settle_s
        while time.time() < t_settle:
            self.read(timeout=0.1)          # 움직이던 때 프레임 버리기
        out = []
        t_end = time.time() + timeout
        while len(out) < n and time.time() < t_end:
            f = self.read(timeout=0.2)
            if f is not None:
                out.append(f)
        return out

    def close(self):
        self.pipeline.set_state(Gst.State.NULL)
