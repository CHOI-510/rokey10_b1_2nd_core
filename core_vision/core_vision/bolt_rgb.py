"""Independent RGB ROI classification; no ROS dependency."""
import numpy as np

SIDES = ('L', 'R')
UNKNOWN = 'UNKNOWN'


def validate(config):
    w, h = config['size']
    r = config['radius']
    if any(type(v) is not int or v <= 0 for v in (w, h, r)):
        raise ValueError('Invalid dimensions')
    if set(config['points']) != set(SIDES):
        raise ValueError('Explicit L/R calibration required')
    for x, y in config['points'].values():
        if type(x) is not int or type(y) is not int or not (r <= x <= w-r and r <= y <= h-r):
            raise ValueError('ROI outside image')
    (x, y), (a, b) = (config['points'][s] for s in SIDES)
    if abs(x-a) < 2*r and abs(y-b) < 2*r:
        raise ValueError('Overlapping ROIs')
    return config


def classify(frame, config):
    if frame.shape != (config['size'][1], config['size'][0], 3):
        raise ValueError('Resolution changed')
    r = config['radius']
    result = []
    for side in SIDES:
        x, y = config['points'][side]
        roi = frame[y-r:y+r, x-r:x+r]
        ratio = np.mean(np.max(roi, axis=2) <= 70)
        # Uniform patches may be occlusions or exposure failures.
        if np.std(roi.astype(float).mean(axis=2)) < 3:
            result.append(UNKNOWN)
        else:
            result.append('PRESENT' if ratio >= .50 else 'MISSING' if ratio <= .20 else UNKNOWN)
    return tuple(result)


def consensus(samples):
    return tuple(col[0] if len(col) >= 5 and len(set(col[-5:])) == 1 else UNKNOWN
                 for col in zip(*samples)) if samples else (UNKNOWN, UNKNOWN)
