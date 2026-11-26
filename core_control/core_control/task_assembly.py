from math import isfinite
from core_common.config import FASTEN_POINTS
from core_common.errors import ErrorCode as Error
from core_common.status_qos import current_stamp, time_ns
from core_common.types import Fastening, Outcome, Presence, Side, State
from core_interfaces.action import Fasten
from core_interfaces.msg import TargetPose
from core_interfaces.srv import InspectPositions, ReadFastening


POSITION_FIELDS = ('presence_l', 'presence_r', 'fastening_l', 'fastening_r',
                   'presence_evidence', 'fastening_evidence')


def position_fields(record):
    return {field: getattr(record, field, '') for field in POSITION_FIELDS}


class TaskEnded(Exception):
    def __init__(self, outcome, reason):
        super().__init__(reason)
        self.outcome = outcome
        self.reason = reason


class PositionTask:
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
        self.presence_l = self.presence_r = Presence.UNKNOWN
        self.fastening_l = self.fastening_r = Fastening.UNKNOWN
        self.presence_evidence = self.fastening_evidence = 'UNVERIFIED'

    def capture_fastening(self, result):
        with self.control.lock:
            if result.state in Fastening._value2member_map_:
                self.fastening = Fastening(result.state)
            self.episode_id = result.episode_id
            self.tightening_valid = result.tightening_valid
            self.peak_torque, self.turns = result.peak_torque, result.turns

    def read_presence(self, stage):
        self.read_evidence(InspectPositions, 'presence', Presence, stage=stage)

    def read_evidence(self, kind, field, enum, **fields):
        with self.control.lock:
            self.control.guard(self)
            setattr(self, field + '_l', enum.UNKNOWN)
            setattr(self, field + '_r', enum.UNKNOWN)
            setattr(self, field + '_evidence', 'UNVERIFIED')
            self.control.publish_status()
        request = kind.Request(task_id=self.task_id, not_before=self.control.stamp(), **fields)
        result = self.control.service(self, kind, request)
        fresh = (time_ns(result.stamp) >= time_ns(request.not_before)
                 and current_stamp(self.control, result.stamp, self.control.evidence_max_age))
        left, right = (getattr(result, field + suffix) for suffix in ('_l', '_r'))
        valid = result.success and result.error_code == Error.OK and fresh
        left_valid, right_valid = (value in enum._value2member_map_ for value in (left, right))
        evidence = 'UNVERIFIED'
        if valid and result.origin == 'FAKE':
            evidence = 'FAKE'
        elif valid and result.origin == 'REAL' and result.verified:
            evidence = 'REAL'
        expected = 'FAKE' if self.control.mode == 'fake' else 'REAL'
        with self.control.lock:
            self.control.guard(self)
            setattr(self, field + '_l', enum(left) if valid and left_valid else enum.UNKNOWN)
            setattr(self, field + '_r', enum(right) if valid and right_valid else enum.UNKNOWN)
            setattr(self, field + '_evidence', evidence)
            if kind is InspectPositions:
                self.control.vision_status = f'{evidence} OK' if evidence == expected else 'UNVERIFIED'
            self.control.log(self.task_id, self.control.state,
                             f'{field}: evidence={evidence}, code={result.error_code}, '
                             f'reason={result.reason}')
            self.control.publish_status()
        if evidence != expected:
            raise TaskEnded(Outcome.HOLD, field + '_evidence_unverified')


class InspectionTask(PositionTask):
    """Collect external evidence without issuing Robot Actions."""

    def run(self):
        self.control.transition(self, State.INSPECT)
        self.read_presence(State.INSPECT)
        if Presence.UNKNOWN in (self.presence_l, self.presence_r):
            raise TaskEnded(Outcome.HOLD, 'position_presence_unknown')
        self.control.transition(self, State.WAIT_WORKER)
        self.control.announce(self, 'L/R 위치를 준비한 뒤 작업자 완료를 알려 주세요.')
        self.control.wait_for(self, lambda: self.done, self.control.worker_timeout,
                              'worker_timeout')
        self.control.transition(self, State.RECHECK)
        self.read_presence(State.RECHECK)
        presence = (self.presence_l, self.presence_r)
        if presence != (Presence.PRESENT, Presence.PRESENT):
            reason = ('position_presence_unknown' if Presence.UNKNOWN in presence
                      else 'position_bolt_missing')
            raise TaskEnded(Outcome.HOLD, reason)

        self.control.transition(self, State.VERIFY)
        self.read_evidence(ReadFastening, 'fastening', Fastening)
        states = (self.fastening_l, self.fastening_r)
        if Fastening.NOT_SEATED in states:
            return Outcome.FAIL, 'position_not_seated'
        if Fastening.UNKNOWN in states:
            return Outcome.HOLD, 'position_fastening_unknown'
        return Outcome.PASS, 'positions_verified_seated'

class FasteningTask(PositionTask):
    """Run the same L/R service and Action sequence with fake or physical providers."""

    def run(self):
        self.control.transition(self, State.INSPECT)
        self.read_presence(State.INSPECT)
        positions = (self.presence_l, self.presence_r)
        if Presence.UNKNOWN in positions:
            raise TaskEnded(Outcome.HOLD, 'position_presence_unknown')
        if Presence.MISSING in positions:
            self.control.transition(self, State.WAIT_WORKER)
            self.control.announce(self, '빠진 볼트를 준비하고 작업 공간을 비운 뒤 WORKER DONE을 눌러 주세요.')
            self.control.wait_for(self, lambda: self.done, self.control.worker_timeout, 'worker_timeout')
            self.control.transition(self, State.RECHECK)
            self.read_presence(State.RECHECK)
            self.require_both_present()

        self.control.transition(self, State.FASTEN)
        self.recipe_id = 'finish'
        simulated = self.control.mode == 'fake'
        for point in FASTEN_POINTS:
            self.control.log(self.task_id, State.FASTEN, f'{point}: finish requested')
            goal = Fasten.Goal(task_id=self.task_id, point_id=point,
                               bolt_pose=TargetPose(), recipe_id=self.recipe_id)
            reply = self.control.action(self, Fasten, goal)
            measured = (reply.success and reply.error_code == Error.OK and reply.tightening_valid
                        and isfinite(reply.peak_torque) and reply.peak_torque > 0
                        and isfinite(reply.turns) and reply.turns > 0 and bool(reply.episode_id))
            seat = Fastening.SEATED if measured and reply.state == Fastening.SEATED else (
                Fastening.NOT_SEATED if reply.state == Fastening.NOT_SEATED else Fastening.UNKNOWN)
            with self.control.lock:
                self.control.guard(self)
                setattr(self, 'fastening_' + point.lower(), seat)
                if seat != Fastening.UNKNOWN:
                    self.fastening_evidence = 'FAKE_ACTION_RESULT' if simulated else 'ACTION_TORQUE'
                self.tightening_valid = seat == Fastening.SEATED
                self.control.log(self.task_id, State.FASTEN,
                                 f'{point}: {seat}, episode={reply.episode_id}, probe_valid={measured}')
                self.control.publish_status()
            if seat == Fastening.NOT_SEATED:
                return Outcome.FAIL, f'{point.lower()}_not_seated'
            if seat != Fastening.SEATED:
                return Outcome.HOLD, f'{point.lower()}_fastening_unconfirmed'
            self.control.wait_for(self, lambda: self.control.terminal_snapshot(self)
                                  and self.control.physical_ready(), self.control.limits['robot_heartbeat'],
                                  'robot_terminal_state_unconfirmed')

        self.control.transition(self, State.VERIFY)
        self.read_presence(State.VERIFY)
        self.require_both_present()
        self.fastening = Fastening.SEATED
        reason = 'lr_present_after_fake_fasten' if simulated else 'lr_present_after_verified_torque'
        return Outcome.PASS, reason

    def require_both_present(self):
        positions = (self.presence_l, self.presence_r)
        if Presence.UNKNOWN in positions:
            raise TaskEnded(Outcome.HOLD, 'position_presence_unknown')
        if Presence.MISSING in positions:
            raise TaskEnded(Outcome.HOLD, 'position_bolt_missing')
