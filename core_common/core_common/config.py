from pathlib import Path

SCENARIOS = ('missing', 'partial', 'seated', 'vision_unknown', 'fasten_failure', 'sensor_mismatch')
FAKE_FRAME = 'base_link'


def data_directory(value):
    return Path(value).expanduser().resolve()
