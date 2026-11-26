#!/usr/bin/env python3
"""Exercise real-mode Control with synthetic Vision and Fasten peers on isolated DDS."""
from tempfile import TemporaryDirectory
from threading import Thread

from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from core_common.ros_context import ros_context
from core_control.control import Control
from core_interfaces.action import Fasten
from core_interfaces.msg import RobotStatus
from core_interfaces.srv import InspectPositions

from fake_integration import Probe


def exercise(probe):
    stages, goals = [], []
    initial = ('PRESENT', 'PRESENT')
    after = ('PRESENT', 'PRESENT')
    origin, unmeasured, completed = 'REAL', '', ''

    def inspect(request, response):
        stages.append(request.stage)
        left, right = {'INSPECT': initial, 'RECHECK': ('PRESENT', 'PRESENT'),
                       'VERIFY': after}[request.stage]
        response.presence_l, response.presence_r = left, right
        response.success, response.verified, response.origin = True, True, origin
        response.stamp = probe.get_clock().now().to_msg()
        return response

    def fasten(handle):
        nonlocal completed
        point = handle.request.point_id
        assert handle.request.recipe_id == 'finish'
        assert not handle.request.bolt_pose.valid
        goals.append(point)
        handle.publish_feedback(Fasten.Feedback(progress=1))
        completed = bytes(handle.goal_id.uuid).hex()
        handle.succeed()
        return Fasten.Result(success=True, state='SEATED', tightening_valid=point != unmeasured,
                             peak_torque=2.0, turns=0.4, episode_id='test-' + point)

    status = probe.create_publisher(RobotStatus, '/robot_status', 10)

    def publish_status():
        status.publish(RobotStatus(stamp=probe.get_clock().now().to_msg(),
                                   robot_mode='real', safe_to_move=True,
                                   last_completed_goal_id=completed))

    probe.create_service(InspectPositions, '/integration/inspect_positions', inspect)
    timer = probe.create_timer(0.05, publish_status)
    server = ActionServer(probe, Fasten, '/fasten', fasten)

    def start():
        stages.clear()
        goals.clear()
        probe.wait(lambda: probe.snapshot and probe.snapshot.mode == 'real'
                   and probe.job.service_is_ready()
                   and 'check' in probe.snapshot.allowed_commands, 'real cycle ready')
        probe.command('check', source='gui')

    def result(index, outcome, reason):
        probe.wait(lambda: len(probe.results) == index, f'cycle result {index}')
        reply = probe.results[-1]
        assert (reply.mode, reply.source, reply.outcome, reply.reason) == (
            'real', 'gui', outcome, reason), reply
        return reply

    try:
        start()
        reply = result(1, 'PASS', 'lr_present_after_verified_torque')
        assert (reply.fastening_l, reply.fastening_r, reply.fastening_evidence) == (
            'SEATED', 'SEATED', 'ACTION_TORQUE')
        assert stages == ['INSPECT', 'VERIFY'] and goals == ['L', 'R']

        initial = ('MISSING', 'PRESENT')
        start()
        probe.wait(lambda: probe.status == 'WAIT_WORKER', 'worker preparation')
        assert goals == []
        probe.command('done', source='gui')
        result(2, 'PASS', 'lr_present_after_verified_torque')
        assert stages == ['INSPECT', 'RECHECK', 'VERIFY'] and goals == ['L', 'R']

        initial, origin = ('PRESENT', 'PRESENT'), 'FAKE'
        start()
        result(3, 'HOLD', 'presence_evidence_unverified')
        assert goals == []

        origin, unmeasured = 'REAL', 'L'
        start()
        reply = result(4, 'HOLD', 'l_fastening_unconfirmed')
        assert goals == ['L'] and stages == ['INSPECT']
        assert reply.fastening_l == reply.fastening_r == 'UNKNOWN'

        unmeasured, after = '', ('PRESENT', 'UNKNOWN')
        start()
        result(5, 'HOLD', 'position_presence_unknown')
        assert stages == ['INSPECT', 'VERIFY'] and goals == ['L', 'R']
    finally:
        server.destroy()
        probe.destroy_timer(timer)


if __name__ == '__main__':
    with ros_context(), TemporaryDirectory(prefix='core-real-cycle-') as data:
        node = Control(parameter_overrides=[Parameter('mode', value='real'),
                                            Parameter('data_dir', value=data)])
        probe = Probe()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = Thread(target=executor.spin)
        thread.start()
        try:
            exercise(probe)
        finally:
            node.shutdown.set()
            executor.shutdown()
            thread.join()
            if node.thread:
                node.thread.join()
            node.journal.close()
            probe.destroy_node()
            node.destroy_node()
    print('PASS real-mode cycle with synthetic ROS peers')
