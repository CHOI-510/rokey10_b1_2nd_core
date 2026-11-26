import json


def feedback(action_type, goal_handle, stage, message, level='INFO'):
    goal_handle.publish_feedback(action_type.Feedback(
        stage=stage, log_level=level, log_msg=message))


def append_jsonl(path, record):
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + '\n')
