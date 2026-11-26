"""Real /fasten backend: runs core_robot.fastening.fasten_core on the Doosan M0609.

Selected by ``mode:=real``. The routine runs under the same
conditions as ``ros2 run core_robot fasten_test --skip-grip``: the robot already holds the
screwdriver and keeps holding it; no hand-over and no gripper open/close.

Goal mapping (checked before any motion)
  recipe_id  'finish'  -> full routine (vision centring/alignment, force contact, tighten until
                          |dtau6| >= TORQUE_DONE_NM with z stalled for DONE_HOLD_STEPS steps)
             'confirm' -> rejected with NOT_IMPLEMENTED (no real "check only" routine yet;
                          it is never replaced by 'finish')
             other     -> INVALID
  point_id   looked up in ``fasten_point_map`` ("point_id:bolt_number"), default
             'assembly_bolt:1'. The bolt number selects a taught pose in
             core_robot.fastening.fasten_core.BOLT_POSES_TAUGHT. Unknown ids -> INVALID.
  bolt_pose  must be a valid TargetPose (same check as Fake). It is NOT used for motion:
             the real routine moves to the taught pose and corrects x/y with the wrist camera.

Result mapping
  torque_done                   success, state SEATED, tightening_valid true
  max_rotation / max_cycles     FAILED, state UNKNOWN, tightening_valid false
  safety abort (z floor, |Fz|)  FAILED, state UNKNOWN
  cancel                        CANCELLED, force released, retreat to view height
  stop / clearance lost         STOPPED, Doosan move_stop requested, force released, no motion
  controller/camera/calib not ready, unsupported goal -> UNAVAILABLE / NOT_IMPLEMENTED / INVALID
                                before any motion
  peak_torque = max |tau6 - tau6 just before tightening| in N·m (0.0 when never measured)
  turns       = counted tool rotation / 360 (seat search + tightening)
"""
from core_common import topics
from core_common.errors import ErrorCode as Error
from core_common.types import Fastening

SUPPORTED_RECIPES = {'finish'}
UNSUPPORTED_RECIPES = {'confirm': 'recipe_confirm_not_implemented_on_real_robot'}
DEFAULT_POINT_MAP = ['assembly_bolt:1']
QUICK_STOP = 1     # dsr_msgs2/srv/MoveStop stop_mode DR_QSTOP (Stop Category 2)


def parse_point_map(entries, taught):
    """['assembly_bolt:1', ...] -> {'assembly_bolt': 1}; every bolt must be a taught pose."""
    mapping = {}
    for entry in entries:
        name, sep, number = str(entry).partition(':')
        if not sep or not name.strip():
            raise ValueError(f'fasten_point_map entry must be "point_id:bolt": {entry!r}')
        bolt = int(number)
        if bolt not in taught:
            raise ValueError(f'fasten_point_map {entry!r}: bolt {bolt} is not taught '
                             f'(taught: {sorted(taught)})')
        mapping[name.strip()] = bolt
    if not mapping:
        raise ValueError('fasten_point_map is empty')
    return mapping


class RealFasten:
    def __init__(self, node, point_map, vision=True, stop_service='', dr=None,
                 dsr_available=None, core=None, camera_topic=None):
        if core is None:
            from .fastening import fasten_core as core
        self.core = core
        self.log = node.get_logger()
        self.point_map = parse_point_map(point_map, core.BOLT_POSES_TAUGHT)
        self.vision = vision
        self.camera_topic = topics.CAMERA_COLOR if vision and camera_topic is None else camera_topic
        self.node = node
        self.fastener = None
        self.last_log = ''
        self.stop_client = None
        self._dsr_available = dsr_available
        if dr is None:
            import rclpy
            from dsr_msgs2.srv import MoveStop, SetRobotMode
            self._move_stop = MoveStop
            # DSR_ROBOT2 spins this node itself on every call, so it must stay out of the executor.
            self.dsr_node = rclpy.create_node('core_robot_dsr', namespace=core.ROBOT_ID)
            self.dr = core.connect_dsr(self.dsr_node)
            self._mode_client = self.dsr_node.create_client(
                SetRobotMode, topics.DSR_SET_ROBOT_MODE)
            if stop_service:
                self.stop_client = node.create_client(MoveStop, stop_service)
        else:
            self.dsr_node = None
            self.dr = dr
            self._mode_client = None
        self.log.info(f'real fasten backend: points {self.point_map}, vision {vision}, '
                      f'camera {self.camera_topic or "disabled"}, stop service {stop_service or "none"}')

    @property
    def force_active(self):
        return bool(self.fastener is not None and self.fastener.force_active)

    def validate(self, goal):
        """(error, reason, bolt) without touching the robot."""
        if goal.recipe_id in UNSUPPORTED_RECIPES:
            return Error.NOT_IMPLEMENTED, UNSUPPORTED_RECIPES[goal.recipe_id], None
        if goal.recipe_id not in SUPPORTED_RECIPES:
            return Error.INVALID, f'unsupported_recipe:{goal.recipe_id}', None
        if goal.point_id not in self.point_map:
            return Error.INVALID, f'unsupported_point_id:{goal.point_id}', None
        return Error.OK, '', self.point_map[goal.point_id]

    def controller_ready(self, timeout=2.0):
        if self._dsr_available is not None:
            return self._dsr_available()
        return self._mode_client.wait_for_service(timeout_sec=timeout)

    def services_ready(self):
        return (self._mode_client.service_is_ready()
                and self.stop_client is not None and self.stop_client.service_is_ready())

    def request_stop(self):
        """Non-blocking: called from robot_node.latch_stop while holding its lock."""
        if self.stop_client is None:
            self.log.warn('real fasten stop: no Doosan move_stop client; relying on routine guard')
            return
        if not self.stop_client.service_is_ready():
            self.log.error('real fasten stop: Doosan move_stop service unavailable')
            return
        self.stop_client.call_async(self._move_stop.Request(stop_mode=QUICK_STOP))
        self.log.warn('real fasten stop: Doosan move_stop (DR_QSTOP) requested')

    def ros_camera(self, check_abort):
        from .fastening.ros_image_camera import RosImageCamera
        return RosImageCamera(self.node, self.camera_topic, check_abort)

    def execute(self, goal, should_abort, notify, result):
        """Run one fastening. Fills result.state/peak_torque/turns/tightening_valid.
        Returns (ErrorCode, reason)."""
        core = self.core
        self.last_log = ''
        result.state = Fastening.UNKNOWN
        result.tightening_valid = False
        result.peak_torque = result.turns = 0.0
        error, reason, bolt = self.validate(goal)
        if error != Error.OK:
            return error, reason
        pending = should_abort()
        if pending:
            return (Error.CANCELLED if pending == 'cancelled' else Error.STOPPED), 'before_motion'
        if not self.controller_ready():
            return Error.UNAVAILABLE, 'dsr_controller_unavailable'
        if bool(self.camera_topic) != self.vision:
            return Error.INVALID, 'ros_camera_and_vision_must_agree'
        notify('PREPARE', f'bolt{bolt}: camera / calibration check ({self.camera_topic or "disabled"})')
        try:
            fastener = core.Fastener(self.dr, self.log, bolt, vision=self.vision,
                                     should_abort=should_abort, on_stage=notify, log_kind='action',
                                     camera_factory=self.ros_camera if self.vision else None)
        except core.Interrupted as exc:
            return (Error.CANCELLED if exc.kind == 'cancelled' else Error.STOPPED), exc.kind
        except core.Abort as exc:
            return Error.UNAVAILABLE, f'fastener_not_ready: {exc}'
        self.fastener = fastener
        self.last_log = fastener.log_path
        motion, error, reason = 'hold', Error.FAILED, ''
        try:
            notify('PREPARE', 'TCP / tool setup')
            fastener.guard()
            core.setup_tool(self.dr, self.log)
            fastener.check_camera()
            fastener.guard()
            notify('PREPARE', 'move to READY_J')
            try:
                core.go_ready(self.dr)
            except Exception as exc:
                error, reason = Error.STOPPED, f'ready_motion_failed: {exc}'
            else:
                motion = 'ready'
                outcome = fastener.run()
                if outcome == core.RESULT_DONE and fastener.torque_samples > 0:
                    error = Error.OK
                    result.state = Fastening.SEATED
                    result.tightening_valid = True
                    reason = outcome
                else:
                    reason = f'tightening_not_confirmed:{outcome}'
        except core.Interrupted as exc:
            if exc.kind == 'stopped':
                error, motion = Error.STOPPED, 'hold'
            else:
                error = Error.CANCELLED
                if motion != 'hold':
                    motion = 'retreat'
            reason = exc.kind
        except core.Abort as exc:
            if motion == 'hold':
                error = Error.UNAVAILABLE
            reason = f'aborted: {exc}'
        except Exception as exc:     # DSR service failure, etc.
            reason = f'exception: {exc}'
        finally:
            try:
                cleaned = fastener.finish(motion)
            except Exception as exc:
                cleaned = False
                self.log.error(f'fastening cleanup: {exc}')
            finally:
                result.turns = fastener.total_deg / 360.0
                result.peak_torque = fastener.max_tau6 if fastener.torque_samples else 0.0
                self.fastener = None
            if not cleaned:
                error, reason = Error.STOPPED, 'fastening_cleanup_failed'
                result.tightening_valid = False
        return error, reason
