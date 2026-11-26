from core_common.types import Presence, Side


def inspect(scenario, stage, view):
    if view == 'top':
        if scenario == 'vision_unknown':
            return Presence.UNKNOWN, Side.UNKNOWN
        if scenario == 'missing' and stage == 'INSPECT':
            return Presence.MISSING, Side.UNKNOWN
        return Presence.PRESENT, Side.UNKNOWN
    if stage == 'VERIFY':
        side = Side.PROTRUDING if scenario == 'sensor_mismatch' else Side.NOT_PROTRUDING
    else:
        side = Side.NOT_PROTRUDING if scenario == 'seated' else Side.PROTRUDING
    return Presence.UNKNOWN, side
