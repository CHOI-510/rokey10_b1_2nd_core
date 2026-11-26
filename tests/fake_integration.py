#!/usr/bin/env python3
"""Run the P0-a scenarios against the installed launch and ROS interfaces."""
import json
import signal
import subprocess
import tempfile
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_interfaces.msg import ControlStatus, RobotStatus, TaskResult
from core_interfaces.srv import EmergencyStop, JobCommand

FULL = ['READY', 'INSPECT', 'FETCH', 'WAIT_WORKER', 'RECHECK', 'STOW', 'FASTEN', 'VERIFY', 'REPORT']
PRESENT = ['READY', 'INSPECT', 'STOW', 'FASTEN', 'VERIFY', 'REPORT']


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


class Probe(Node):
    def __init__(self):
        super().__init__('fake_integration_probe')
        self.status = None
        self.snapshot = None
        self.robot_status = None
        self.results = []
        self.create_subscription(ControlStatus, '/control_status', self.on_status, 10)
        self.create_subscription(RobotStatus, '/robot_status', self.on_robot_status, 10)
        self.create_subscription(TaskResult, '/task_result', self.results.append, 10)
        self.job = self.create_client(JobCommand, '/job_command')
        self.stop = self.create_client(EmergencyStop, '/emergency_stop')

    def on_status(self, msg):
        self.status = msg.state
        self.snapshot = msg

    def on_robot_status(self, msg):
        self.robot_status = msg

    def wait(self, predicate, description, timeout=15):
        deadline = monotonic() + timeout
        while not predicate():
            assert monotonic() < deadline, f'Timed out waiting for {description}, state={self.status}'
            rclpy.spin_once(self, timeout_sec=0.05)

    def command(self, command, accepted=True, source='cli'):
        if command == 'stop':
            future = self.stop.call_async(EmergencyStop.Request(command='stop', source=source))
        else:
            future = self.job.call_async(JobCommand.Request(command=command, source=source))
        self.wait(future.done, command, 5)
        reply = future.result()
        if accepted is not None:
            assert (reply.success if command == 'stop' else reply.accepted) == accepted, reply
        return reply


def run_case(label, scenario, outcome, reason, states, recipe=None,
             interruption=None, missing_node=None):
    with tempfile.TemporaryDirectory(prefix='core-fake-') as directory:
        data = Path(directory)
        with (data / 'launch.log').open('w+') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'core_bringup', 'system.launch.py', 'mode:=fake',
                 f'scenario:={scenario}', f'data_dir:={data}', 'llm:=false'],
                stdout=log, stderr=subprocess.STDOUT,
            )
            probe = Probe()
            try:
                probe.wait(lambda: probe.job.service_is_ready() and probe.stop.service_is_ready()
                           and probe.status == 'READY' and probe.robot_status is not None
                           and probe.robot_status.safe_to_move, 'system ready')
                if missing_node:
                    assert missing_node not in dict(probe.get_node_names_and_namespaces())
                if label == 'LLM offline':
                    completed = subprocess.run(
                        ['ros2', 'run', 'core_control', 'control_interface', 'check'],
                        capture_output=True, text=True, timeout=8)
                    assert completed.returncode == 0, completed
                else:
                    probe.command('check')
                if scenario == 'missing':
                    probe.wait(lambda: probe.status == 'WAIT_WORKER', 'operator handoff')
                    if interruption:
                        probe.command(interruption)
                    else:
                        probe.command('done')
                probe.wait(lambda: len(probe.results) == 1, 'task result')
                result = probe.results[0]
                assert (result.outcome, result.reason, list(result.states)) == (outcome, reason, states), result
                probe.wait(lambda: (data / 'task_results.jsonl').exists()
                           and (data / 'task_results.jsonl').read_text().endswith('\n'),
                           'asynchronous result journal')
                persisted, = read_jsonl(data / 'task_results.jsonl')
                assert persisted['task_id'] == result.task_id and persisted['outcome'] == outcome
                assert persisted['stop_reason'] == result.stop_reason
                assert persisted['source'] == result.source == 'cli'
                events = read_jsonl(data / 'event_logs.jsonl')
                assert any(event['task_id'] == result.task_id for event in events)
                if recipe:
                    episode = data / 'episodes' / result.episode_id
                    probe.wait(lambda: (episode / 'result.json').exists(), 'episode flush')
                    metadata = json.loads((episode / 'metadata.json').read_text())
                    assert metadata['task_id'] == result.task_id and metadata['mode'] == 'fake'
                    episode_result = json.loads((episode / 'result.json').read_text())
                    assert episode_result['success'] == (outcome != 'FAIL'), episode_result
                    assert result.tightening_valid
                    assert result.recipe_id == recipe
                    assert (result.peak_torque, result.turns) == (
                        episode_result['peak_torque_nm'], episode_result['turns'])
                    assert recipe in (episode / 'trajectory.csv').read_text()
                else:
                    assert not result.episode_id
                if interruption == 'stop':
                    probe.wait(lambda: probe.status == 'STOPPED' and probe.robot_status.stopped,
                               'both stop latches')
                    for _ in range(3):
                        probe.command('check', accepted=False)
                        assert probe.status == 'STOPPED'
                    def resume_confirmed():
                        reply = probe.command('resume', accepted=None)
                        assert reply.accepted or reply.reason == 'awaiting_robot_stop_confirmation', reply
                        return reply.accepted

                    probe.wait(resume_confirmed, 'explicit resume')
                probe.wait(lambda: probe.status == 'READY', 'return to ready')
                if interruption == 'stop':
                    assert len(probe.results) == 1
                print(f'PASS {label}', flush=True)
            except Exception:
                log.seek(0)
                print(log.read(), flush=True)
                raise
            finally:
                probe.destroy_node()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


def main():
    cases = [
        ('missing bolt', 'missing', 'PASS', 'verified_seated', FULL, 'finish'),
        ('partially seated', 'partial', 'PASS', 'verified_seated', PRESENT, 'finish'),
        ('fully seated', 'seated', 'PASS', 'verified_seated', PRESENT, 'confirm'),
        ('unknown vision', 'vision_unknown', 'HOLD', 'bolt_presence_or_pose_unconfirmed',
         ['READY', 'INSPECT', 'REPORT']),
        ('fastening failure', 'fasten_failure', 'FAIL', 'fastening_failed',
         [*PRESENT[:4], 'REPORT'], 'finish'),
        ('sensor mismatch', 'sensor_mismatch', 'HOLD', 'verification_sensor_mismatch_or_unknown',
         PRESENT, 'finish'),
        ('operator stop', 'missing', 'HOLD', 'operator_stop', [*FULL[:4], 'STOPPED'], None, 'stop'),
        ('DB offline', 'partial', 'PASS', 'verified_seated', PRESENT, 'finish', None, 'database_node'),
        ('LLM offline', 'partial', 'PASS', 'verified_seated', PRESENT, 'finish', None, 'llm_node'),
        ('operator cancel', 'missing', 'HOLD', 'cancelled', [*FULL[:4], 'REPORT'], None, 'cancel'),
    ]
    with ros_context():
        for case in cases:
            run_case(*case)


if __name__ == '__main__':
    main()
