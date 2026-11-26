import argparse
import json
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from core_common import topics
from .bolt_rgb import validate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default=topics.CAMERA_COLOR)
    parser.add_argument('--output', default='~/.config/core_vision/vision1.json')
    parser.add_argument('--radius', type=int, default=35)
    args = parser.parse_args()
    rclpy.init()
    node = Node('vision1_calibrate')
    bridge = CvBridge()
    frame = None
    points = {}
    def receive(msg):
        nonlocal frame
        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            frame = None
    sub = node.create_subscription(Image, args.topic, receive, qos_profile_sensor_data)
    window = 'Vision1 calibration'
    cv2.namedWindow(window)
    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and frame is not None:
            if 'L' not in points:
                points['L'] = [x,y]
            elif 'R' not in points:
                points['R'] = [x,y]
    cv2.setMouseCallback(window, click)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if frame is not None:
                image = frame.copy()
                cv2.putText(image, 'Click L (Bolt 2), then R (Bolt 1). S save / C clear / Q quit', (5,25), 0,.45,(0,255,255),1)
                for side,(x,y) in points.items():
                    r = args.radius
                    cv2.rectangle(image,(x-r,y-r),(x+r,y+r),(0,255,255),2)
                    cv2.putText(image,side,(x,y),0,.7,(0,255,255),2)
                cv2.imshow(window,image)
            key = cv2.waitKey(1)&255
            if key == ord('q'): break
            if key == ord('c'): points.clear()
            if key == ord('s') and frame is not None:
                try:
                    config = validate(dict(size=list(frame.shape[1::-1]), radius=args.radius, points=points, topic=args.topic))
                    path = Path(args.output).expanduser()
                    path.parent.mkdir(parents=True,exist_ok=True)
                    path.write_text(json.dumps(config,indent=2))
                    print(path)
                    break
                except Exception as exc:
                    print(exc)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
