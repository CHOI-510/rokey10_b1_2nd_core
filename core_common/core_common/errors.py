from enum import IntEnum


class ErrorCode(IntEnum):
    OK = 0
    INVALID = 1
    BUSY = 2
    CANCELLED = 3
    STOPPED = 4
    TIMEOUT = 5
    UNAVAILABLE = 6
    FAILED = 7
    NOT_IMPLEMENTED = 8
