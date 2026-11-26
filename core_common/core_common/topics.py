"""ROS names shared by CORE publishers, subscribers, service and action peers."""

# Topics
CAMERA_COLOR = '/camera/camera/color/image_raw'
CONTROL_STATUS = '/control_status'
ROBOT_STATUS = '/robot_status'
EVENT_LOG = '/event_log'
TASK_RESULT = '/task_result'
ANNOUNCE = '/announce'

# Services
JOB_COMMAND = '/job_command'
EMERGENCY_STOP = '/emergency_stop'
ROBOT_EMERGENCY_STOP = '/robot/emergency_stop'
INSPECT = '/inspect'
LOCATE = '/locate'
INSPECT_POSITIONS = '/integration/inspect_positions'
READ_FASTENING = '/integration/read_fastening'
PARSE_COMMAND = '/parse_command'
SUMMARIZE = '/summarize'

# Actions
MOVE_TO = '/move_to'
FETCH = '/fetch'
STOW = '/stow'
FASTEN = '/fasten'

# Existing Doosan names
DSR_ROBOT_ID = 'dsr01'
DSR_SET_ROBOT_MODE = f'/{DSR_ROBOT_ID}/dsr_controller2/system/set_robot_mode'
DSR_MOVE_STOP = f'/{DSR_ROBOT_ID}/dsr_controller2/motion/move_stop'
