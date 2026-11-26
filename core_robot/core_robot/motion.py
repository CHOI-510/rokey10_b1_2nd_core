from core_common.config import FAKE_FRAME
from core_common.robot_api import RobotAPI
from core_interfaces.msg import TargetPose

VIEWS = {'top': 0.35, 'side': 0.25}


class FakeRobot(RobotAPI):
    def __init__(self, clock, motion_permission):
        self.clock = clock
        self.motion_permission = motion_permission

    def safe_to_move(self):
        return self.motion_permission()

    def move_to(self, view_id):
        target = TargetPose(valid=True)
        target.pose.header.frame_id = FAKE_FRAME
        target.pose.header.stamp = self.clock.now().to_msg()
        target.pose.pose.position.z = VIEWS[view_id]
        target.pose.pose.orientation.w = 1.0
        return target
