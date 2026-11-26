from core_common.config import FAKE_FRAME
from core_interfaces.msg import TargetPose


def locate(item_id, stamp):
    target = TargetPose()
    coordinates = {'bolt': (0.25, 0.10), 'driver': (0.35, -0.10)}
    if item_id in coordinates:
        target.valid = True
        target.pose.header.frame_id = FAKE_FRAME
        target.pose.header.stamp = stamp
        target.pose.pose.position.x, target.pose.pose.position.y = coordinates[item_id]
        target.pose.pose.position.z = 0.1
        target.pose.pose.orientation.w = 1.0
    return target
