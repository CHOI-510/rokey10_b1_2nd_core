import json
from queue import Full, Queue
from threading import Thread


def feedback(action_type, goal_handle, stage, message, level='INFO', progress=0):
    goal_handle.publish_feedback(action_type.Feedback(
        stage=stage, log_level=level, log_msg=message, progress=progress))


def append_jsonl(path, record):
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + '\n')


class Journal:
    """Write records on a bounded worker and drain accepted writes at shutdown."""

    def __init__(self, path, on_error, capacity=1024):
        self.path = path
        self.on_error = on_error
        self.queue = Queue(maxsize=capacity)
        self.thread = Thread(target=self.write, name='core-journal')
        self.thread.start()

    def submit(self, filename, record):
        self.enqueue(filename, append_jsonl, self.path / filename, record)

    def enqueue(self, label, operation, *args):
        try:
            self.queue.put_nowait((label, operation, args))
        except Full:
            self.on_error(f'{label}: record lost (journal queue full)')

    def write(self):
        while (entry := self.queue.get()) is not None:
            label, operation, args = entry
            try:
                operation(*args)
            except Exception as exc:
                self.on_error(f'{label}: record lost or incomplete ({exc})')

    def close(self):
        self.queue.put(None)
        self.thread.join()
