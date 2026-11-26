# ROS 2 통신 계약 v0.1

## 좌표와 시간

`TargetPose`는 `geometry_msgs/PoseStamped pose`와 `bool valid`를 포함합니다. Fake의 프레임은 `base_link`이고 위치는 m, 회전은 정규화한 XYZW 쿼터니언입니다. 유효한 좌표에는 ROS 시각의 `header.stamp`와 프레임이 필요합니다. `valid=false`인 값은 이동 목표에 사용할 수 없습니다. 토픽 시각은 ROS 시계의 `builtin_interfaces/Time`입니다. 경과 시간과 타임아웃은 단조 시계를 사용합니다.

## Actions

모든 Goal에 `string task_id`가 포함됩니다. Result 공통 필드는 `bool success`, `int32 error_code`이며 Feedback은 `string stage, log_level, log_msg`입니다. ID, 상태, 레시피는 문자열입니다.

| 엔드포인트와 타입 | Goal 추가 필드 | Result 추가 필드 |
|---|---|---|
| /move_to · MoveTo | view_id (top, side) | TargetPose current_pose |
| /fetch · Fetch | item_id, TargetPose pick_pose, dest_id | |
| /stow · Stow | item_id, TargetPose grasp_pose, holder_id | |
| /fasten · Fasten | point_id, TargetPose bolt_pose, recipe_id (finish, confirm) | state, float64 peak_torque (N·m), float64 turns (회전수), episode_id |

Robot Node가 Action 공통 실행 슬롯을 관리합니다. 이미 실행 중이거나 이동 허가가 없을 때는 ROS Goal을 거절합니다. 거절된 Goal에는 Result가 발행되지 않습니다.

## Services

| 엔드포인트와 타입 | Request | Response |
|---|---|---|
| /inspect · Inspect | task_id, stage, view_id | success, error_code, presence, side, TargetPose bolt_pose |
| /locate · Locate | task_id, item_id | success, error_code, TargetPose pose |
| /job_command · JobCommand | command | bool accepted, state, task_id, reason |
| /parse_command · ParseCommand | text | success, error_code, command |
| /summarize · Summarize | TaskResult result | success, error_code, text |
| /emergency_stop · EmergencyStop | command=stop | success, error_code, reason |
| /robot/emergency_stop · EmergencyStop (노드 내부 연동) | command=stop 또는 resume | success, error_code, reason |

`check`는 READY에서 작업을 시작하고, `done`은 WAIT_WORKER에서 작업자 완료를 전달합니다. `cancel`은 활성 작업을 취소합니다. `resume`은 STOPPED에서 복구를 요청하며 READY 발행으로 복구 완료를 알립니다. 외부 정지 Ack는 관제 래치 적용을 의미합니다. 관제는 별도로 로봇 정지 응답을 확인합니다. 복귀 서비스가 요청을 수락해도, 로봇 복귀 응답과 이동 가능한 유휴 상태를 확인한 뒤 관제가 READY를 발행합니다. P0-a의 ParseCommand와 Summarize는 선택 노드에서 NOT_IMPLEMENTED를 반환합니다.

## Topics

아래 `core_interfaces` 메시지에 `stamp`가 포함됩니다. Task 관련 메시지는 동일한 `task_id`로 연결합니다.

| Topic | 메시지 | 추가 필드 |
|---|---|---|
| /control_status | ControlStatus | task_id, state |
| /robot_status | RobotStatus | bool busy, stopped, safe_to_move |
| /event_log | LogEvent | task_id, stage, log_level, log_msg |
| /task_result | TaskResult | task_id, outcome, reason, fastening, side, episode_id, string[] states |
| /announce | Announcement | task_id, text |

판정은 `core_common/types.py`의 Presence, Side, Fastening, Outcome 문자열을 그대로 사용합니다. `reason`은 결과에 필수입니다. 오류 코드의 기준은 `core_common/errors.py`에 있습니다.

| 값 | 이름 |
|---|---|
| 0 | OK |
| 1 | INVALID |
| 2 | BUSY |
| 3 | CANCELLED |
| 4 | STOPPED |
| 5 | TIMEOUT |
| 6 | UNAVAILABLE |
| 7 | FAILED |
| 8 | NOT_IMPLEMENTED |

## Episode 기록

`data/episodes/EP<UUID>/`에 `metadata.json`, `trajectory.csv`, `result.json`을 저장합니다. 메타데이터는 schema_version, episode_id, task_id, stamp_ns, frame_id, orientation, units, clock, mode를 포함합니다. 결과는 episode_id, stamp_ns, success, reason, fastening, peak_torque_nm, turns를 포함합니다.

CSV의 열은 `core_robot/episode_recorder.py`의 `COLUMNS`에 정의되어 있습니다. 시각은 ROS 나노초, 관절은 rad와 rad/s, TCP 위치는 m, 자세는 XYZW 쿼터니언, 힘·모멘트는 N·N·m, 강성은 N/m 또는 N·m/rad입니다. 각 행은 FSM 상태, 체결 Stage, 실제 명령, 목표 힘과 강성을 담을 수 있습니다. Fake에서는 수집할 수 없는 물리량을 빈 셀로 두고 체결 시작·종료 Stage와 사용한 레시피를 기록합니다. 선택적 `rosbag/`은 동일 Episode ID와 ROS 시간으로 RGB-D, CameraInfo, TF, IMU를 연결합니다.

## 관리자 GUI

`monitor_gui`는 다음 엔드포인트를 사용합니다.

| 구독 또는 호출 | 타입 | 화면 및 요청 |
|---|---|---|
| `/control_status` | `core_interfaces/msg/ControlStatus` | FSM, 작업 ID, 버튼 활성 상태와 STOPPED → READY 확인 |
| `/robot_status` | `core_interfaces/msg/RobotStatus` | 로봇의 READY, BUSY, STOPPED, BLOCKED 표시 |
| `/event_log` | `core_interfaces/msg/LogEvent` | 진행 단계, 실시간 검사·체결 판정과 로그 |
| `/task_result` | `core_interfaces/msg/TaskResult` | 최종 결과·사유, 측면 및 체결 판정, 방문한 단계 |
| `/job_command` | `core_interfaces/srv/JobCommand` | CHECK → `check`, WORKER DONE → `done`, RESUME → `resume`, CANCEL → `cancel` |
| `/emergency_stop` | `core_interfaces/srv/EmergencyStop` | STOP → `stop` |

관제는 기존 `LogEvent.log_msg`에 `inspection_top=<presence>`, `inspection_side=<side>`, `fastening=<state>` 이벤트를 추가합니다. GUI는 작업 ID로 표시를 연결하고 최종 판정을 `/task_result`에서 가져옵니다. 네 상태·이벤트 구독은 휘발성으로 동작하며 GUI가 구독 후 수신한 결과와 이벤트를 표시합니다.

관제나 로봇 상태가 1초 이상 갱신되지 않으면 해당 상태를 NO SIGNAL로 표시합니다. 관제 상태가 끊기면 작업 버튼은 비활성화되고 STOP 요청은 계속 가능합니다. 서비스가 발견되지 않으면 `service_unavailable`, 응답이 5초 안에 오지 않으면 `response_timeout; outcome unknown`을 이벤트 로그에 표시합니다.

### 선택 센서 입력

| 용도 | 파라미터 | 기본값 | 표준 메시지 |
|---|---|---|---|
| 손목 영상 | `wrist_image_topic` | `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| 측면 영상 | `side_image_topic` | 빈 문자열 | `sensor_msgs/msg/Image` |
| 체결 중 힘·모멘트 | `wrench_topic` | 빈 문자열 | `geometry_msgs/msg/WrenchStamped` |

영상과 체결 센서의 구독은 Best Effort, Keep Last 1을 사용합니다. 카메라 토픽이 비어 있으면 OFF, 유효한 프레임이 오면 LIVE, 유효한 프레임을 일정 시간 받지 못하면 NO SIGNAL로 표시합니다. GUI 노드의 `camera_timeout` 파라미터 기본값은 2.0초입니다. `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1`을 지원하며, 발행 측이 영상에 그린 YOLO 박스도 그대로 표시할 수 있습니다.

체결 입력은 공구 좌표계의 Z축으로 정렬한 Fz(N)와 Mz(N·m)를 사용합니다. 발행 측은 관제와 같은 ROS 시간 기준으로 `header.stamp`를 채웁니다. GUI는 최신 관제 상태가 FASTEN인 동안 체결 단계 시작 시각 이후의 샘플을 수집합니다. 그래프에는 최근 400개 샘플을 그리며, 최대값은 수신한 샘플에서 구한 각 축의 최대 절댓값입니다. 체결 시간은 첫·마지막 샘플 사이의 관측 구간입니다. 샘플이 없으면 N/A가 표시됩니다. GUI의 `mode:=fake`에서는 수신값에 MOCK을 붙이며, 이 표시는 GUI 모드 설정에 따릅니다. 기본 Fake Launch는 영상이나 Wrench를 발행하지 않습니다.
