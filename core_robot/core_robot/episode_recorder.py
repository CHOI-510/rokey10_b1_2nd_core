import csv
import json
from pathlib import Path
from uuid import uuid4

COLUMNS = (
    'stamp_ns', 'state', 'stage',
    *(f'joint_{i}_rad' for i in range(1, 7)),
    *(f'joint_{i}_rad_s' for i in range(1, 7)),
    'tcp_x_m', 'tcp_y_m', 'tcp_z_m', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw',
    'fx_n', 'fy_n', 'fz_n', 'mx_nm', 'my_nm', 'mz_nm',
    'command', 'target_force_n', 'stiffness_n_m', 'stiffness_nm_rad',
)


class EpisodeRecorder:
    def __init__(self, root: Path, task_id, stamp_ns, frame_id, mode):
        self.episode_id = f'EP{uuid4().hex}'
        self.path = root / self.episode_id
        self.path.mkdir(parents=True)
        self._write('metadata.json', dict(
            schema_version=1, episode_id=self.episode_id, task_id=task_id,
            stamp_ns=stamp_ns, frame_id=frame_id, mode=mode,
            orientation='quaternion_xyzw', units='SI', clock='ROS'))
        self._stream = (self.path / 'trajectory.csv').open('w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(self._stream, fieldnames=COLUMNS)
        self._writer.writeheader()

    def _write(self, filename, value):
        target = self.path / filename
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
        temporary.replace(target)

    def sample(self, **sample):
        self._writer.writerow(sample)
        self._stream.flush()

    def finish(self, stamp_ns, success, reason, **details):
        self._stream.close()
        self._write('result.json', dict(episode_id=self.episode_id, stamp_ns=stamp_ns,
                                       success=success, reason=reason, **details))
