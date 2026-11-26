#!/usr/bin/env python3
"""Exercise the inspection-only launch against synthetic, read-only providers."""
import signal
import subprocess
import sys
from contextlib import nullcontext
from pathlib import Path
from tempfile import TemporaryDirectory

from core_common.ros_context import ros_context
from core_interfaces.srv import InspectPositions, ReadFastening

from fake_integration import Probe, read_jsonl


def exercise(probe, data, window=None, wait=None):
    presence = {'INSPECT': ('PRESENT', 'MISSING'), 'RECHECK': ('PRESENT', 'PRESENT')}
    fastening = ('SEATED', 'NOT_SEATED')
    origin = 'REAL'  # Exercises the admitted contract branch with synthetic data in this test only.
    force_calls = []

    def inspect(request, response):
        response.presence_l, response.presence_r = presence[request.stage]
        response.success, response.verified, response.origin = True, True, origin
        response.stamp = probe.get_clock().now().to_msg()
        return response

    def read_fastening(request, response):
        force_calls.append(request.task_id)
        response.fastening_l, response.fastening_r = fastening
        response.success, response.verified, response.origin = True, True, origin
        response.stamp = probe.get_clock().now().to_msg()
        return response

    probe.create_service(InspectPositions, '/integration/inspect_positions', inspect)
    probe.create_service(ReadFastening, '/integration/read_fastening', read_fastening)
    probe.wait(lambda: probe.snapshot and probe.snapshot.mode == 'inspection'
               and probe.job.service_is_ready(), 'inspection launch')
    assert {'robot_node', 'vision_node'}.isdisjoint(
        name for name, _ in probe.get_node_names_and_namespaces())

    job = probe.command('check', source='voice')
    probe.wait(lambda: probe.status == 'WAIT_WORKER', 'manual preparation')
    assert (probe.snapshot.presence_l, probe.snapshot.presence_r) == ('PRESENT', 'MISSING')
    assert not force_calls
    if window:
        wait(lambda: window.values['top']['text'] == 'PRESENT / UNKNOWN', 'L in GUI')
        assert window.values['side']['text'] == 'MISSING / UNKNOWN'
        assert window.captions['top']['text'] == 'Left(L) 존재 / 체결'
        assert window.captions['side']['text'] == 'Right(R) 존재 / 체결'
    probe.command('done', source='voice')
    probe.wait(lambda: len(probe.results) == 1, 'independent fastening result')
    result = probe.results[-1]
    assert result.task_id == job.task_id and result.source == 'voice'
    assert (result.outcome, result.fastening_l, result.fastening_r) == ('FAIL', 'SEATED', 'NOT_SEATED')
    assert list(result.states) == ['READY', 'INSPECT', 'WAIT_WORKER', 'RECHECK', 'VERIFY', 'REPORT']
    if window:
        wait(lambda: window.values['side']['text'] == 'PRESENT / NOT_SEATED', 'R fastening in GUI')
    assert force_calls == [job.task_id]

    origin = 'FAKE'
    probe.command('check')
    probe.wait(lambda: len(probe.results) == 2, 'Fake evidence is held')
    assert probe.results[-1].outcome == 'HOLD' and probe.results[-1].presence_evidence == 'FAKE'
    assert force_calls == [job.task_id]

    origin = 'REAL'
    presence['INSPECT'] = ('UNKNOWN', 'PRESENT')
    probe.command('check')
    probe.wait(lambda: len(probe.results) == 3, 'unknown is independent')
    assert (probe.results[-1].presence_l, probe.results[-1].presence_r) == ('UNKNOWN', 'PRESENT')
    assert probe.results[-1].outcome == 'HOLD'

    presence['INSPECT'] = ('PRESENT', 'PRESENT')
    fastening = ('SEATED', 'SEATED')
    probe.command('check')
    probe.wait(lambda: probe.status == 'WAIT_WORKER', 'prepare all-present positions')
    probe.command('done')
    probe.wait(lambda: len(probe.results) == 4, 'verified result')
    assert probe.results[-1].outcome == 'PASS'

    probe.command('check')
    probe.wait(lambda: probe.status == 'WAIT_WORKER', 'prepare before STOP')
    assert probe.command('stop').reason == 'inspection_stop_latched'
    probe.wait(lambda: len(probe.results) == 5, 'stop result')
    probe.command('check', accepted=False)
    probe.command('resume', accepted=False, source='voice')
    probe.command('resume')

    def journal_written():
        file = Path(data) / 'task_results.jsonl'
        return file.exists() and len(file.read_text().splitlines()) == 5

    probe.wait(journal_written, 'persist all results')
    records = read_jsonl(Path(data) / 'task_results.jsonl')
    for key in ('presence_l', 'presence_r', 'fastening_l', 'fastening_r'):
        assert records[0][key] == getattr(result, key)
    assert records[4]['stop_reason'] == 'operator_stop'


def main():
    with ros_context(), TemporaryDirectory(prefix='core-lf-') as data:
        process = subprocess.Popen(['ros2', 'launch', 'core_bringup', 'system.launch.py',
                                    'mode:=inspection', f'data_dir:={data}'])
        probe = Probe()
        try:
            display = nullcontext((None, None, None))
            if '--gui' in sys.argv:
                from monitor_integration import monitor
                display = monitor(probe, mode='inspection', preview_enabled=False)
            with display as (_, window, wait):
                exercise(probe, data, window, wait)
        finally:
            probe.destroy_node()
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
    print('PASS inspection integration', flush=True)


if __name__ == '__main__':
    main()
