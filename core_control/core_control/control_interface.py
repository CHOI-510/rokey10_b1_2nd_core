import argparse

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from core_common.ros_context import ros_context
from core_interfaces.srv import EmergencyStop, JobCommand


def main():
    parser = argparse.ArgumentParser(description='Core Project job commands')
    parser.add_argument('command', choices=('check', 'done', 'cancel', 'resume', 'stop'))
    args = parser.parse_args(remove_ros_args()[1:])
    with ros_context():
        node = Node('job_cli')
        try:
            stop = args.command == 'stop'
            interface, name = (EmergencyStop, '/emergency_stop') if stop else (JobCommand, '/job_command')
            client = node.create_client(interface, name)
            if not client.wait_for_service(timeout_sec=3):
                parser.exit(1, f'{name} unavailable\n')
            future = client.call_async(interface.Request(command=args.command))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if not future.done():
                parser.exit(1, 'Command response timed out\n')
            reply = future.result()
            print(reply)
            if not (reply.success if stop else reply.accepted):
                raise SystemExit(1)
        finally:
            node.destroy_node()
