# ROS 2 통신 계약 v0.1

## 좌표와 시간

`TargetPose`는 `geometry_msgs/PoseStamped pose`와 `bool valid`를 포함합니다. Fake의 프레임은 `base_link`이고 위치는 m, 회전은 정규화한 XYZW 쿼터니언입니다. 유효한 좌표에는 ROS 시각의 `header.stamp`와 프레임이 필요합니다. `valid=false`인 값은 이동 목표에 사용할 수 없습니다. 토픽 시각은 ROS 시계의 `builtin_interfaces/Time`입니다. 경과 시간과 타임아웃은 단조 시계를 사용합니다.

## Actions

모든 Goal에 `string task_id`가 포함됩니다. Result 공통 필드는 `bool success`, `int32 error_code`, `string stop_reason`이며 Feedback은 `string stage, log_level, log_msg`, `uint32 progress`입니다. 반복 Feedback은 통신 수신만 확인합니다. `progress`가 증가해야 진행 감시가 갱신됩니다. ID, 상태, 레시피는 문자열입니다.

| 엔드포인트와 타입 | Goal 추가 필드 | Result 추가 필드 |
|---|---|---|
| /move_to · MoveTo | view_id (top, side) | TargetPose current_pose |
| /fetch · Fetch | item_id, TargetPose pick_pose, dest_id | |
| /stow · Stow | item_id, TargetPose grasp_pose, holder_id | |
| /fasten · Fasten | point_id, TargetPose bolt_pose, recipe_id (finish, confirm) | state, float64 peak_torque (N·m), float64 turns (회전수), tightening_valid, episode_id |

Robot Node가 Action 공통 실행 슬롯을 관리합니다. 이미 실행 중이거나 이동 허가가 없을 때는 ROS Goal을 거절합니다. 거절된 Goal에는 Result가 발행되지 않습니다.

## Services

| 엔드포인트와 타입 | Request | Response |
|---|---|---|
| /inspect · Inspect | task_id, stage, view_id | success, error_code, presence, side, TargetPose bolt_pose |
| /locate · Locate | task_id, item_id | success, error_code, TargetPose pose |
| /job_command · JobCommand | command, source | bool accepted, state, task_id, reason |
| /parse_command · ParseCommand | text | success, error_code, command, confidence_valid, confidence, needs_confirmation |
| /summarize · Summarize | TaskResult result | success, error_code, text |
| /emergency_stop · EmergencyStop | command=stop, source | success, error_code, reason |
| /robot/emergency_stop · EmergencyStop (노드 내부 연동) | command=stop 또는 resume, source, reason, stop_id, expires_at | success, error_code, reason |

`JobCommand.source`는 `gui`, `cli`, `voice` 중 하나입니다. `check`는 검증된 READY에서, `done`은 WAIT_WORKER에서, `cancel`은 활성 작업에서만 허용됩니다. `resume`은 관리자 조작 경로(GUI/CLI)에서 기존 작업 종료와 안전 확인 후 새 작업 준비를 요청합니다. GUI 및 CLI의 출처 필드는 기능상 출처 구분용이며 ROS 서비스 인증을 구현하지 않습니다. `stop`은 별도의 EmergencyStop 서비스로 요청합니다.

정지 서비스의 성공은 요청한 계층의 래치 접수입니다. `RobotStatus.stop_confirmed`는 Fake 동작 종료를 확인하는 필드입니다. Robot은 중복 `stop_id`를 제한된 이력에 기억하고 서로 다른 직접 STOP 요청에는 새 래치 ID를 적용합니다. 이어서 관제의 복구 요청은 현재 래치 ID, 만료 시각, 최신 STOPPED 상태와 물리 조건을 검사합니다. 관제는 성공 응답 및 후속 Robot 상태 모두를 확인해 READY로 전환합니다. 응답 유실·상태 불일치 시 재래치하고 새 관리자 요청을 기다립니다.

### 음성 담당자 계약

Wakeword → STT → `/parse_command` → 로컬 allowlist와 최신 `allowed_commands` 확인 → `/job_command` (`source=voice`). `core_language/voice_commands.py`는 클라이언트 템플릿입니다. 해석 응답의 `confidence_valid=false`는 점수가 없음을 뜻하며 명시 확인이 필요합니다. `needs_confirmation=true`도 확인 절차를 요구합니다. 음성 허용 명령은 `check`, `done`, `cancel`입니다. 로컬에서 정지 의도를 식별하면 `request_local_stop()`으로 두 경로를 호출합니다. 음성으로 래치를 해제하지 않습니다.

`/announce`는 공구 인계 및 WAIT_WORKER, 완료 접수, 검사 결과와 작업 판정의 한국어 템플릿을 제공하며 콘텐츠는 `core_common/announcements.py`에 있습니다. ParseCommand와 Summarize는 현재 NOT_IMPLEMENTED를 반환합니다. 음성 프런트엔드는 기한을 적용하고 파싱 실패에는 `PARSE_UNAVAILABLE`, 요약 실패나 Timeout에는 원래 `/announce` 문구를 사용합니다.

## Topics

아래 `core_interfaces` 메시지에 `stamp`가 포함됩니다. Task 관련 메시지는 동일한 `task_id`로 연결합니다.

| Topic | 메시지 | 추가 필드 |
|---|---|---|
| /control_status | ControlStatus | mode, control_id, state, task_id, last_task_id, recovery, stop_reason, top, side, fastening, outcome, last_reason, allowed_commands, states, vision_status |
| /robot_status | RobotStatus | robot_mode, busy, stopped, stop_confirmed, safe_to_move, stop_reason, stop_id, error_code, error_message, gripper_valid, gripper_width (m), grip_detected, holding_item, current_action, current_pose, force_state_known, force_control_active, compliance_control_active, last_completed_goal_id |
| /event_log | LogEvent | task_id, node, source, stage, log_level, log_msg |
| /task_result | TaskResult | task_id, mode, source, outcome, reason, stop_reason, top, side, fastening, recipe_id, episode_id, tightening_valid, peak_torque, turns, states |
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

GUI는 `/control_status`에서 판정과 허용 명령을 읽습니다. STOP은 `/emergency_stop`과 `/robot/emergency_stop`을 독립 호출합니다. `/control_status`와 `/task_result`는 Reliable, Transient Local, Keep Last 1이며 GUI 구독에도 같은 QoS를 적용합니다. Robot의 Heartbeat 구독은 호환 가능한 Volatile로 새 발행만 받습니다. 새 구독자는 살아 있는 발행기의 마지막 작업 결과를 받습니다. 재시작 이후의 이력은 JSONL을 사용합니다. 진행 중이면 `task_id`와 같은 결과만 표시하고 유휴 시에는 `last_task_id`로 연결합니다. `tightening_valid`가 참일 때만 최종 토크와 회전수를 표시합니다. 실시간 Wrench 그래프는 관측값입니다.

Control과 Robot 상태에 메시지 시각 및 로컬 수신 기한을 적용합니다. 기본 1초 동안 갱신이 없으면 NO SIGNAL로 표시하며 작업 버튼을 비활성화합니다. Robot STOP은 계속 요청할 수 있습니다. 요청 경로 미발견은 `service_unavailable`, GUI 응답 기한 5초 초과는 `response_timeout`으로 기록합니다. Vision의 초기 UNVERIFIED와 UNAVAILABLE은 서비스 발견 여부를 반영하고, FAKE OK·NOT_IMPLEMENTED·ERROR·TIMEOUT은 실제 호출 결과를 반영합니다.

### 선택 센서 입력

| 용도 | 파라미터 | 기본값 | 표준 메시지 |
|---|---|---|---|
| 손목 영상 | `wrist_image_topic`, `wrist_image_type` | `/camera/color/image_raw`, `raw` | Image 또는 CompressedImage |
| 측면 영상 | `side_image_topic`, `side_image_type` | 빈 문자열, `raw` | Image 또는 CompressedImage |
| 체결 중 힘·모멘트 | `wrench_topic` | 빈 문자열 | `geometry_msgs/msg/WrenchStamped` |

영상과 체결 센서의 구독은 Best Effort, Keep Last 1을 사용합니다. 카메라 토픽이 비어 있으면 OFF, 유효한 프레임이 오면 LIVE, 유효한 프레임을 일정 시간 받지 못하면 NO SIGNAL로 표시합니다. GUI 노드의 `camera_timeout` 파라미터 기본값은 2.0초입니다. `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1`을 지원하며, 발행 측이 영상에 그린 YOLO 박스도 그대로 표시할 수 있습니다.

체결 입력은 공구 좌표계의 Z축으로 정렬한 Fz(N)와 Mz(N·m)를 사용합니다. 발행 측은 관제와 같은 ROS 시간 기준으로 `header.stamp`를 채웁니다. GUI는 최신 관제 상태가 FASTEN인 동안 체결 단계 시작 시각 이후의 샘플을 수집합니다. 그래프에는 최근 400개 샘플을 그리며, 최대값은 수신한 샘플에서 구한 각 축의 최대 절댓값입니다. 체결 시간은 첫·마지막 샘플 사이의 관측 구간입니다. 샘플이 없으면 N/A가 표시됩니다. GUI의 `mode:=fake`에서는 수신값에 MOCK을 붙이며, 이 표시는 GUI 모드 설정에 따릅니다. 기본 Fake Launch는 영상이나 Wrench를 발행하지 않습니다.

영상 연결, 설정과 측정 방법은 [영상 파이프라인](VIDEO_PIPELINE.md)에 있습니다.
