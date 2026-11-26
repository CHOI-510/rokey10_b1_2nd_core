"""Client template for a future local voice frontend. No speech or model runtime."""

from uuid import uuid4

from core_common import topics
from core_interfaces.srv import EmergencyStop, JobCommand, ParseCommand

from .prompts import ALLOWED_COMMANDS


class VoiceCommands:
    def __init__(self, node):
        self.parse = node.create_client(ParseCommand, topics.PARSE_COMMAND)
        self.jobs = node.create_client(JobCommand, topics.JOB_COMMAND)
        self.stop = node.create_client(EmergencyStop, topics.EMERGENCY_STOP)
        self.robot_stop = node.create_client(EmergencyStop, topics.ROBOT_EMERGENCY_STOP)

    def interpret(self, transcript):
        """The caller must apply a bounded wait and give a local failure prompt."""
        return self.parse.call_async(ParseCommand.Request(text=transcript))

    def submit(self, interpretation, *, allowed_commands, minimum_confidence, confirmed=False):
        if (not interpretation.success or interpretation.command not in ALLOWED_COMMANDS
                or interpretation.command not in allowed_commands
                or (interpretation.confidence_valid
                    and not 0.0 <= minimum_confidence <= interpretation.confidence <= 1.0)
                or ((interpretation.needs_confirmation or not interpretation.confidence_valid)
                    and not confirmed)):
            return None
        return self.jobs.call_async(JobCommand.Request(command=interpretation.command, source='voice'))

    def request_local_stop(self):
        """Call after a locally recognized stop intent, independently of ParseCommand."""
        stop_id = uuid4().hex
        pending = {}
        for name, client in (('control', self.stop), ('robot_node', self.robot_stop)):
            try:
                if client.service_is_ready():
                    pending[name] = client.call_async(
                        EmergencyStop.Request(command='stop', source='voice', stop_id=stop_id))
            except Exception as exc:
                pending[name] = exc
        return pending
