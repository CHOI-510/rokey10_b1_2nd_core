from enum import StrEnum


class Presence(StrEnum):
    MISSING = 'MISSING'
    PRESENT = 'PRESENT'
    UNKNOWN = 'UNKNOWN'


class Side(StrEnum):
    PROTRUDING = 'PROTRUDING'
    NOT_PROTRUDING = 'NOT_PROTRUDING'
    UNKNOWN = 'UNKNOWN'


class Fastening(StrEnum):
    SEATED = 'SEATED'
    NOT_SEATED = 'NOT_SEATED'
    UNKNOWN = 'UNKNOWN'


class Outcome(StrEnum):
    PASS = 'PASS'
    FAIL = 'FAIL'
    HOLD = 'HOLD'


class State(StrEnum):
    READY = 'READY'
    INSPECT = 'INSPECT'
    FETCH = 'FETCH'
    WAIT_WORKER = 'WAIT_WORKER'
    RECHECK = 'RECHECK'
    STOW = 'STOW'
    FASTEN = 'FASTEN'
    VERIFY = 'VERIFY'
    REPORT = 'REPORT'
    STOPPED = 'STOPPED'
