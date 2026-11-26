from time import monotonic

from core_common.announcements import FETCH_COMPLETE, HANDOFF_COMPLETE, INSPECTION
from core_common.errors import ErrorCode as Error
from core_common.types import Fastening, Outcome, Presence, Side, State
from core_interfaces.action import Fasten, Fetch, MoveTo, Stow
from core_interfaces.srv import Inspect, Locate


class TaskEnded(Exception):
    def __init__(self, outcome, reason):
        super().__init__(reason)
        self.outcome = outcome
        self.reason = reason


class AssemblyTask:
    def __init__(self, control, task_id, source):
        self.control = control
        self.task_id = task_id
        self.source = source
        self.top = Presence.UNKNOWN
        self.outcome = ''
        self.recipe_id = ''
        self.tightening_valid = False
        self.peak_torque = self.turns = 0.0
        self.states = [State.READY]
        self.done = False
        self.cancelled = False
        self.fastening = Fastening.UNKNOWN
        self.side = Side.UNKNOWN
        self.episode_id = ''
        self.last_goal_id = ''

    def run(self):
        self.control.transition(self, State.INSPECT)
        top = self.inspect('top')
        if top.presence == Presence.MISSING:
            self.control.transition(self, State.FETCH)
            for item in ('bolt', 'driver'):
                self.action(Fetch, item_id=item, pick_pose=self.locate(item), dest_id='handoff')
            self.control.transition(self, State.WAIT_WORKER)
            self.control.announce(self, FETCH_COMPLETE)
            self.control.wait_for(self, lambda: self.done, self.control.worker_timeout,
                                  'worker_timeout')
            self.control.announce(self, HANDOFF_COMPLETE)
            until = monotonic() + 3
            self.control.wait_for(self, lambda: monotonic() >= until, 4, 'handoff_delay_timeout')
            self.control.require_clearance()
            self.control.transition(self, State.RECHECK)
            top = self.inspect('top')

        if top.presence != Presence.PRESENT or not top.bolt_pose.valid:
            raise TaskEnded(Outcome.HOLD, 'bolt_presence_or_pose_unconfirmed')
        self.side = self.inspect('side').side
        if self.side not in (Side.PROTRUDING, Side.NOT_PROTRUDING):
            raise TaskEnded(Outcome.HOLD, 'side_inspection_unknown')

        self.control.transition(self, State.STOW)
        self.action(Stow, item_id='driver', grasp_pose=self.locate('driver'), holder_id='driver_holder')
        self.control.transition(self, State.FASTEN)
        self.recipe_id = 'confirm' if self.side == Side.NOT_PROTRUDING else 'finish'
        result = self.action(Fasten, point_id='assembly_bolt', bolt_pose=top.bolt_pose, recipe_id=self.recipe_id)
        if self.fastening == Fastening.NOT_SEATED:
            raise TaskEnded(Outcome.FAIL, 'fastening_failed')
        if not result.success:
            raise TaskEnded(Outcome.HOLD, f'fasten_{Error(result.error_code).name.lower()}')
        if self.fastening != Fastening.SEATED:
            raise TaskEnded(Outcome.HOLD, 'fastening_unknown')
        self.control.transition(self, State.VERIFY)
        self.side = self.inspect('side').side
        if self.side != Side.NOT_PROTRUDING:
            raise TaskEnded(Outcome.HOLD, 'verification_sensor_mismatch_or_unknown')
        return Outcome.PASS, 'verified_seated'

    def capture_fastening(self, result):
        with self.control.lock:
            if result.state in Fastening._value2member_map_:
                self.fastening = Fastening(result.state)
            self.episode_id = result.episode_id
            self.tightening_valid = result.tightening_valid
            self.peak_torque, self.turns = result.peak_torque, result.turns

    def action(self, kind, **fields):
        result = self.control.action(self, kind, kind.Goal(task_id=self.task_id, **fields))
        if kind is not Fasten and not result.success:
            raise TaskEnded(Outcome.HOLD, f'{kind.__name__.lower()}_{Error(result.error_code).name.lower()}')
        return result

    def inspect(self, view):
        self.action(MoveTo, view_id=view)
        result = self.control.service(self, Inspect, Inspect.Request(
            task_id=self.task_id, stage=self.states[-1], view_id=view))
        if not result.success:
            raise TaskEnded(Outcome.HOLD, 'inspect_failed')
        with self.control.lock:
            if view == 'top':
                self.top = Presence(result.presence)
            else:
                self.side = Side(result.side)
            self.control.publish_status()
        self.control.announce(self, INSPECTION[view].format(result.presence if view == 'top' else result.side))
        return result

    def locate(self, item):
        result = self.control.service(self, Locate, Locate.Request(task_id=self.task_id, item_id=item))
        if not result.success or not result.pose.valid:
            raise TaskEnded(Outcome.HOLD, f'locate_{item}_failed')
        return result.pose
