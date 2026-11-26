# 협동-2 초기 코어 설계 v0.1 (Design Freeze, 2026-11-26)

## 환경
Ubuntu 24.04 / ROS 2 Jazzy / M0609 + RG2(Modbus TCP) / 손목 D435i / YOLO·OpenCV / Wakeword·STT·LLM API·TTS / Git main 기준

## 코어 시퀀스 (FSM, 단일 Task)
READY → INSPECT(상단: MISSING/PRESENT/UNKNOWN)
→ [MISSING] FETCH(볼트 트레이, 드라이버 → 인계대) → WAIT_WORKER("다 했어" + 3초 + 안내) → RECHECK(상단)
→ 측면 검사 → STOW(드라이버 탐색·거치대 꽂기) → FASTEN(마무리 체결 또는 확인 조임, 항상 실행)
→ VERIFY(측면) → REPORT(JSONL + /task_result + TTS)
- UNKNOWN → HOLD. STOPPED는 자동 재가동 금지, 명시적 복구 필요.
- 드라이버 거치대 = 보관 위치 + 재파지 스테이션. 육각 소켓 비트 사용.

## 판정 Enum (core_common/types.py 한 곳)
- 존재: MISSING / PRESENT / UNKNOWN
- 측면: PROTRUDING / NOT_PROTRUDING / UNKNOWN
- 체결: SEATED / NOT_SEATED / UNKNOWN
- 최종: PASS / FAIL / HOLD (+ reason 필수)

| 체결 | 측면 | 최종 |
|---|---|---|
| SEATED | NOT_PROTRUDING | PASS |
| SEATED | PROTRUDING / UNKNOWN | HOLD |
| NOT_SEATED | 무관 | FAIL |
| UNKNOWN | 무관 | HOLD |

## 노드·인터페이스
- control(FSM, 유일 허브), robot_node(로봇 독점, Action 배타 실행), vision_node(Service), voice_node, llm_node, monitor_gui, database_node(선택)
- Action: MoveTo(view_id→current_pose), Fetch(item_id, pick_pose, dest_id), Stow(item_id, grasp_pose, holder_id), Fasten(point_id, bolt_pose, recipe_id → state, peak_torque, turns, episode_id). 공통 success, error_code / Feedback stage, log_level, log_msg
- Service: Inspect, Locate, JobCommand(check/done/cancel/resume), ParseCommand, Summarize, EmergencyStop
- Topic: /control_status, /robot_status, /event_log, /task_result, /announce
- LLM은 입구(의도 파싱)·출구(요약)만. 작업 순서·안전은 관제가 결정. CLI 경로는 시연까지 유지.
- DB는 /task_result 구독만(관제는 DB를 기다리지 않음). 로컬 JSONL이 원본.

## 데이터
data/task_results.jsonl, event_logs.jsonl, episodes/EPxxx/{metadata.json, trajectory.csv, result.json}
trajectory: t, TCP xyz/자세, J1~J6, F/M 6축, target force, stiffness, commanded action, stage

## 패키지 (필수 7)
core_interfaces, core_common, core_control, core_robot, core_vision, core_language, core_bringup (+ 선택 core_database, web_ui)

## v0.1 설계 동결 시점의 우선순위

현재 구현과 남은 작업은 [로드맵](ROADMAP.md)을 참고합니다.
P0-a 관제 코어(Fake 9단계 CLI 완주), JSONL 및 Episode 계약 → P0-b 실물 시퀀스 → P0-c 음성·LLM → P1 GUI 및 Episode 검증 → P2 DB/Web UI, Task Queue, 글로벌 카메라, IL/RL

## 첫날 완료 기준 (Fake)
누락 / 반체결 / 완전체결 / 비전 판정 불가 / 체결 실패 / 센서 불일치 / 작업 중 정지 / DB 미실행 / LLM 미실행 — 9개 시나리오 모두 기대 결과대로 종료

## P0-a 적용 사항

- 볼트 누락 경로에서는 RECHECK에서 상단과 측면을 검사한다. 볼트가 있는 경로에서는 INSPECT에서 상단과 측면을 검사한다.
- 정지 시 STOPPED 상태를 유지하며 최초 원인과 결과를 기록한다. 물리 상태가 불확실하면 RECOVER에서 관리자 확인을 기다린다. 안전 조건과 동작 종료가 확인되면 명시적 RESUME 이후 새 작업을 받을 수 있다.
- 관제 스냅샷과 마지막 작업 결과를 DDS로 제공한다. GUI는 기존 ROS 영상 토픽을 Preview로 표시하며, Vision Input 및 기존 RTP/JPEG Unicast를 독립적으로 운영한다.
- 필드, 실행 정책과 변경 사유는 INTERFACES.md와 DEVELOPMENT.md에 기록한다.


## P0-b L/R 결과 통합

- `mode:=fake`는 위 단일 볼트 조립 시퀀스를 실행합니다.
- `mode:=inspection`은 같은 패키지와 FSM의 상태를 재사용합니다. L/R별 존재 검사를 받고 작업자 준비 후 재검사합니다. 두 볼트가 확인되면 VERIFY에서 외부 체결 검사 결과를 읽습니다.
- L/R 검사 모드에서 Robot Action, 자동 공구 인계·거치 및 마무리 조임은 실행 범위에 포함되지 않습니다. 서비스 계약, 판정 근거와 실패 처리 기준은 [INTERFACES.md](INTERFACES.md#lr-최소-통합-계약)에 있습니다.
