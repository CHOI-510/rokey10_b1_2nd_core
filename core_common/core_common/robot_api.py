from abc import ABC, abstractmethod


class RobotAPI(ABC):
    @abstractmethod
    def move_to(self, view_id):
        """Return a valid stamped TCP pose on success."""

    @abstractmethod
    def request_stop(self):
        """Submit stop independently of blocking motion. Confirmation comes from telemetry."""

    @abstractmethod
    def safe_to_move(self):
        """Return current motion permission. Unknown must return False."""
