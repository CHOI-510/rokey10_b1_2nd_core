import rclpy
from rclpy.node import Node

from core_common.ros_context import ros_context
from core_interfaces.msg import ControlStatus, LogEvent, RobotStatus, TaskResult


def main():
    import tkinter as tk

    with ros_context():
        node = Node('monitor_gui')
        root = tk.Tk()
        root.title('Core Project')
        control = tk.StringVar(value='Control waiting for status')
        robot = tk.StringVar(value='Robot waiting for status')
        tk.Label(root, textvariable=control).pack(anchor='w')
        tk.Label(root, textvariable=robot).pack(anchor='w')
        journal = tk.Listbox(root, width=90, height=16)
        journal.pack(fill='both', expand=True)

        def append(line):
            journal.insert(tk.END, line)
            if journal.size() > 200:
                journal.delete(0)
            journal.yview_moveto(1)

        node.create_subscription(ControlStatus, '/control_status',
                                 lambda m: control.set(f'Control {m.state} {m.task_id}'), 10)
        node.create_subscription(RobotStatus, '/robot_status', lambda m: robot.set(
            f'Robot busy={m.busy} stopped={m.stopped} safe_to_move={m.safe_to_move}'), 10)
        node.create_subscription(LogEvent, '/event_log',
                                 lambda m: append(f'{m.stage} {m.log_level} {m.log_msg}'), 10)
        node.create_subscription(TaskResult, '/task_result',
                                 lambda m: append(f'{m.outcome} {m.reason} {m.task_id}'), 10)

        def poll():
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
                root.after(20, poll)
            else:
                root.destroy()

        try:
            root.after(20, poll)
            root.mainloop()
        finally:
            node.destroy_node()
