#!/usr/bin/env python3
"""Real /fasten backend checks that need no robot, camera or Doosan controller.

A. Routine contract: RealFasten + core_robot.fastening.fasten_core against a simulated DSR API
   (success, not confirmed, unmeasured torque, cancel, stop, unsupported goals).
B. ROS Action: robot_node with fasten_backend:=real on local DDS, real DSR_ROBOT2 import but no
   controller -> goal validation results, UNAVAILABLE before motion, duplicate /fasten guard.

Run from the repository root after sourcing the Doosan workspace and the Core overlay:
    python3 tests/real_fasten_contract.py
It never drives a robot: part A uses a simulated DSR object, part B has no controller to talk to.
"""
import os
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from core_common.errors import ErrorCode as Error
from core_common.types import Fastening


# ----------------------------------------------------------------- A. routine contract (no ROS)
class SimDR:
    """Minimal DSR_ROBOT2 stand-in: force-controlled descent, a screw that seats after
    `seat_deg` of rotation, then rising J6 torque. Records every motion command."""
    DR_BASE, DR_TOOL, DR_MV_MOD_REL, DR_FC_MOD_REL = 0, 1, 1, 1
    ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS = 0, 1

    def __init__(self, seat_deg=90.0, torque=True, contact_z=13.0, advance=1.0):
        self.x = [393.42, 69.65, 100.0, 122.92, 179.98, 123.93]
        self.j = [10.0, -10.0, 80.0, 0.0, 110.0, 0.0]
        self.force = False
        self.contact_z = contact_z
        self.turned = 0.0
        self.seat_deg = seat_deg
        self.torque = torque
        self.advance = advance              # mm the screw advances per rotation step until seated
        self.motions = []

    # motion
    def movel(self, pos, vel=None, acc=None, ref=None, mod=0, **_):
        self.motions.append(('movel', tuple(pos), mod))
        if mod == self.DR_MV_MOD_REL:
            for i in range(6):
                self.x[i] += pos[i]
            self.j[5] += pos[5]
            if self.force and pos[5]:
                self.turned += abs(pos[5])
                if self.turned < self.seat_deg:
                    self.contact_z -= self.advance   # screw advancing
                    self.x[2] = self.contact_z
        else:
            self.x = list(pos)

    def movej(self, pos, vel=None, acc=None, mod=0, **_):
        self.motions.append(('movej', tuple(pos), mod))
        if mod == self.DR_MV_MOD_REL:
            self.j = [a + b for a, b in zip(self.j, pos)]
        else:
            self.j = list(pos)

    # state
    def get_current_posx(self, *_):
        if self.force and self.x[2] > self.contact_z:
            self.x[2] = max(self.contact_z, self.x[2] - 2.0)
        return [list(self.x), 0]

    def get_current_posj(self):
        return list(self.j)

    def get_tool_force(self, *_):
        touching = self.force and self.x[2] <= self.contact_z + 1e-6
        return [0.0, 0.0, 2.2 + (5.0 if touching else 0.0), 0.0, 0.0, 0.0]

    def get_external_torque(self):
        if not self.torque:
            return -1
        tau = 0.3 + (2.0 if self.turned >= self.seat_deg else 0.0)
        return [0.0] * 5 + [tau]

    # force / setup
    def task_compliance_ctrl(self, *_):
        pass

    def set_desired_force(self, *_, **__):
        self.force = True

    def release_force(self, *_):
        self.force = False

    def release_compliance_ctrl(self, *_):
        pass

    def set_tool(self, *_):
        return 0

    def set_robot_mode(self, *_):
        return 0

    def add_tcp(self, *_):
        return 0

    def set_tcp(self, *_):
        return 0

    def get_tcp(self):
        return 'Driver_Tip_v1'


class Log:
    def info(self, *_): pass
    def warn(self, *_): pass
    def error(self, *_): pass


def goal(recipe='finish', point='assembly_bolt'):
    return SimpleNamespace(recipe_id=recipe, point_id=point)


def result():
    return SimpleNamespace(state='', tightening_valid=None, peak_torque=None, turns=None)


def make(dr, ready=True):
    from core_robot.fastening import fasten_core as core
    from core_robot.real_fasten import RealFasten
    node = SimpleNamespace(get_logger=lambda: Log())
    return RealFasten(node, ['assembly_bolt:1'], vision=False, dr=dr,
                      dsr_available=lambda: ready, core=core)


def check(label, condition):
    if not condition:
        raise AssertionError(label)
    print('PASS', label)


def routine_cases(log_dir):
    from core_robot.fastening import fasten_core as core
    core.LOG_DIR = log_dir
    core.time.sleep = lambda *_: None          # simulated robot: no real waiting
    notes = []

    def never():
        return None

    for recipe, point, code in (('confirm', 'assembly_bolt', Error.NOT_IMPLEMENTED),
                                ('polish', 'assembly_bolt', Error.INVALID),
                                ('finish', 'bolt_9', Error.INVALID)):
        dr = SimDR()
        r = result()
        error, reason = make(dr).execute(goal(recipe, point), never, lambda *a: notes.append(a), r)
        check(f'{recipe}/{point} -> {code.name} before motion ({reason})',
              error == code and not dr.motions and r.state == Fastening.UNKNOWN
              and r.tightening_valid is False and r.turns == 0.0)

    dr = SimDR()
    r = result()
    error, reason = make(dr, ready=False).execute(goal(), never, lambda *a: None, r)
    check('controller unavailable -> UNAVAILABLE before motion',
          error == Error.UNAVAILABLE and not dr.motions and r.tightening_valid is False)

    dr = SimDR(seat_deg=90.0)
    r = result()
    error, reason = make(dr).execute(goal(), never, lambda *a: notes.append(a), r)
    check(f'finish seats -> OK SEATED valid, turns {r.turns:.2f}, peak {r.peak_torque:.2f} N·m',
          error == Error.OK and r.state == Fastening.SEATED and r.tightening_valid
          and r.turns > 0 and r.peak_torque >= core.TORQUE_DONE_NM
          and dr.motions[-1][0] == 'movej' and dr.motions[-1][2] == 0)       # back to READY_J
    check('stage feedback reported', any(n[0] == 'TIGHTEN' for n in notes))

    dr = SimDR(seat_deg=1e9, advance=0.0)
    r = result()
    error, reason = make(dr).execute(goal(), never, lambda *a: None, r)
    check(f'never seats -> FAILED UNKNOWN not valid ({reason})',
          error == Error.FAILED and r.state == Fastening.UNKNOWN and not r.tightening_valid
          and r.turns >= core.MAX_TOTAL_DEG / 360.0)

    dr = SimDR(seat_deg=90.0, torque=False)
    r = result()
    error, reason = make(dr).execute(goal(), never, lambda *a: None, r)
    check(f'torque unreadable -> not success, peak 0.0 ({reason})',
          error != Error.OK and not r.tightening_valid and r.peak_torque == 0.0)

    for kind, code in (('cancelled', Error.CANCELLED), ('stopped', Error.STOPPED)):
        dr = SimDR(seat_deg=1e9, advance=0.0)
        calls = {'n': 0, 'at': None}

        def abort_later(kind=kind, calls=calls, dr=dr):
            calls['n'] += 1
            if calls['n'] > 40:
                if calls['at'] is None:
                    calls['at'] = len(dr.motions)
                return kind
            return None

        r = result()
        error, reason = make(dr).execute(goal(), abort_later, lambda *a: None, r)
        after = dr.motions[calls['at']:]
        if kind == 'stopped':
            ok = not after                                   # no motion once stop is seen
        else:
            ok = all(m[0] == 'movel' and m[1][:2] == (0, 0) and m[1][2] > 0 for m in after)
            ok = ok and not any(m[0] == 'movej' for m in after)   # retreat up only, no READY_J
        check(f'{kind} mid-run -> {code.name}, {"no motion" if kind == "stopped" else "vertical retreat only"} '
              f'after request ({len(after)} motions)',
              error == code and ok and not r.tightening_valid and r.state == Fastening.UNKNOWN)


# ----------------------------------------------------------------- B. ROS Action (subprocess)
ACTION_SCRIPT = r'''
import sys
from threading import Thread
from time import monotonic, sleep
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from core_common.errors import ErrorCode as Error
from core_interfaces.action import Fasten
from core_robot.robot_node import RobotNode

def params(**kw):
    return {"enable_rosout": False, "parameter_overrides": [Parameter(k, value=v) for k, v in kw.items()]}

rclpy.init()
data = sys.argv[1]
node = RobotNode(**params(data_dir=data, fasten_backend="real", require_control_heartbeat=False,
                          fasten_vision=False))
client_node = rclpy.create_node("real_fasten_probe")
client = ActionClient(client_node, Fasten, "/fasten")
ex = MultiThreadedExecutor(num_threads=4); ex.add_node(node); ex.add_node(client_node)
Thread(target=ex.spin, daemon=True).start()
assert client.wait_for_server(timeout_sec=10.0), "no /fasten server"

def send(recipe="finish", point="assembly_bolt", valid=True):
    g = Fasten.Goal(task_id="T", point_id=point, recipe_id=recipe)
    g.bolt_pose.valid = valid
    g.bolt_pose.pose.header.frame_id = "base_link"
    g.bolt_pose.pose.header.stamp = node.get_clock().now().to_msg()
    g.bolt_pose.pose.pose.orientation.w = 1.0
    fb = []
    f = client.send_goal_async(g, feedback_callback=lambda m: fb.append(m.feedback))
    end = monotonic() + 10
    while not f.done() and monotonic() < end: sleep(0.02)
    h = f.result()
    if not h.accepted:
        return None, fb
    r = h.get_result_async()
    end = monotonic() + 30
    while not r.done() and monotonic() < end: sleep(0.05)
    return r.result().result, fb

def show(label, res, fb):
    if res is None:
        print(f"RESULT {label} REJECTED")
    else:
        print(f"RESULT {label} success={res.success} code={Error(res.error_code).name} state={res.state} "
              f"valid={res.tightening_valid} peak={res.peak_torque} turns={res.turns} "
              f"episode={bool(res.episode_id)} feedback={len(fb)} last={fb[-1].log_msg if fb else ''}")

show("confirm", *send("confirm"))
show("unknown_point", *send(point="bolt_9"))
show("invalid_pose", *send(valid=False))
show("finish_no_controller", *send())
if sys.argv[2] == "duplicate":
    fake = RobotNode(**params(data_dir=data, require_control_heartbeat=False))
    ex.add_node(fake)
    sleep(1.5)
    before = node.last_completed_goal_id
    send()   # the client may get either server's reply; judge the real node by what it executed
    sleep(1.0)
    print(f"RESULT real_executed_with_duplicate {node.last_completed_goal_id != before}")
    fake.journal.close()
ex.shutdown(); node.journal.close(); rclpy.shutdown()
'''


def action_cases(data):
    env = dict(os.environ)
    out = subprocess.run([sys.executable, '-c', ACTION_SCRIPT, data, 'duplicate'],
                         capture_output=True, text=True, timeout=120, env=env)
    lines = {line.split()[1]: line for line in out.stdout.splitlines() if line.startswith('RESULT ')}
    if out.returncode != 0 or len(lines) < 5:
        print(out.stdout[-3000:], out.stderr[-3000:])
        raise AssertionError('action subprocess failed')
    for label, line in lines.items():
        print('   ', line)
    check('Action confirm -> NOT_IMPLEMENTED, not success, UNKNOWN',
          'success=False code=NOT_IMPLEMENTED state=UNKNOWN valid=False' in lines['confirm'])
    check('Action unknown point_id -> INVALID',
          'success=False code=INVALID' in lines['unknown_point'])
    check('Action invalid bolt_pose -> INVALID',
          'success=False code=INVALID' in lines['invalid_pose'])
    check('Action finish without Doosan controller -> UNAVAILABLE, not valid, episode recorded',
          'success=False code=UNAVAILABLE state=UNKNOWN valid=False peak=0.0 turns=0.0 episode=True'
          in lines['finish_no_controller'])
    check('real robot_node does not execute while a second /fasten server (Fake) is in the graph',
          lines['real_executed_with_duplicate'].endswith('False'))

    script = r'''
import sys, rclpy
from rclpy.parameter import Parameter
from core_robot.robot_node import RobotNode
from time import sleep
p = lambda **kw: {"enable_rosout": False, "parameter_overrides": [Parameter(k, value=v) for k, v in kw.items()]}
rclpy.init()
fake = RobotNode(**p(data_dir=sys.argv[1], require_control_heartbeat=False))
sleep(1.0)
try:
    real = RobotNode(**p(data_dir=sys.argv[1], fasten_backend="real", require_control_heartbeat=False, fasten_vision=False))
    print("STARTED")
    real.journal.close()
except RuntimeError as exc:
    print("REFUSED", exc)
fake.journal.close()
rclpy.shutdown()
'''
    out = subprocess.run([sys.executable, '-c', script, data], capture_output=True, text=True,
                         timeout=60, env=env)
    check('real robot_node refuses to start while Fake /fasten exists',
          'REFUSED /fasten is already served' in out.stdout)


if __name__ == '__main__':
    with TemporaryDirectory(prefix='core-real-fasten-') as tmp:
        routine_cases(tmp)
        action_cases(tmp)
