from abc import ABC, abstractmethod


class GripperAPI(ABC):
    @abstractmethod
    def command(self, width_m, force_n):
        """Command calibrated opening in metres and grip force in newtons."""

    @abstractmethod
    def status(self):
        """Return measured width in metres and the grip-confirmation flag."""


class FakeGripper(GripperAPI):
    def __init__(self):
        self.width_m = 0.1
        self.gripped = False

    def command(self, width_m, force_n):
        self.width_m = width_m
        self.gripped = width_m == 0 and force_n > 0

    def status(self):
        return self.width_m, self.gripped
