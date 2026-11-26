# 관제 코어 개발 가이드

## 환경 조사 및 재사용

Ubuntu 24.04.5와 ROS 2 Jazzy, colcon, Cyclone DDS를 확인했습니다. 기존 `~/ws_cobot_pjt/ws_dsr`에는 `dsr_msgs2`, `m0609_rg2_bringup`, OnRobot 패키지가 있고 `~/cobot2_ws`에는 협동-2 샘플 패키지의 기존 빌드가 있습니다. P0-a는 Jazzy만 source해서 빌드할 수 있습니다.

협동-1의 `box_comm/dsr_io.py`에서 두산 이동·상태·정지·순응·힘 서비스와 정리 순서를 확인했습니다. `robot.py`의 TCP/Tool 적용, `gripper.py`의 RG2 요청·상태 확인, `part_server.py`의 Feedback 및 Reentrant 그룹/Executor를 검토했습니다. 이번 단계에는 Feedback과 Action 취소·Executor 패턴을 적용했습니다. 두산 서비스와 RG2 통신은 다음 단계 어댑터 후보로 두었습니다. 박스 작업의 좌표, Travel Z, 재고 정책, Recovery는 이번 공정에 적용하지 않았습니다. [협동-1 소스](https://github.com/CHOI-510/rokey10_b1_1st_project)

Gazebo 시뮬레이션 저장소는 참조로 제공되었으며 원격 페이지 확인에 실패했습니다. 이번 검증은 ROS Action·Service를 통해 Fake 알고리즘을 실행합니다. [시뮬레이션 저장소](https://github.com/CHOI-510/m0609-rg2-simulation)

## 담당자별 개발

아래 명령은 README의 환경을 설정하고 통합 Launch를 종료한 뒤 실행합니다.

| 담당 | 입력과 출력 | 성공·오류 기준 |
|---|---|---|
| motion | view_id → TargetPose | 뷰 이동 및 유효한 TCP 측정. 잘못된 뷰는 INVALID |
| fetch | item_id, pick_pose, dest_id → success | 집기 및 인계 확인. 잘못된 목표는 INVALID, 집기 실패는 FAILED |
| stow | item_id, grasp_pose, holder_id → success | 거치와 재파지 준비 확인. 잘못된 목표는 INVALID |
| fasten | point_id, bolt_pose, recipe_id → 상태, 토크, 회전수, Episode ID | SEATED 또는 NOT_SEATED/UNKNOWN과 실패 사유. 확인 조임과 마무리 조임 레시피를 별도로 검증 |
| camera / detector | RGB-D, CameraInfo, 시각, 모델 설정 → 탐지 후보 | 캡처·모델 실패와 판정 불확실성을 호출자에게 전달 |
| transform / locate / inspect | 타임스탬프 좌표, TF, 물품 ID 또는 뷰 → TargetPose 또는 판정 | 시각에 맞는 좌표 변환. 관측 또는 변환 실패 시 UNKNOWN/invalid |
| wakeword / STT / TTS | 오디오·설정 또는 안내 텍스트 → 활성화·텍스트·재생 | 모델 또는 재생 실패를 호출자에게 전달 |
| LLM | 텍스트 또는 TaskResult → 허용 명령 또는 요약 | 허용 목록 밖 명령 및 API 실패를 오류로 반환 |
| control / common / interfaces / bringup | ROS 계약, Fake 시나리오, 설정 → 상태·결과 | 통합 테스트로 상태 전이, 결과, 로그와 Episode 기록 확인 |

### 단독 실행

```bash
# 로봇 담당. 각각 다른 터미널에서 실행
ros2 run core_robot robot_node --ros-args -p mode:=fake -p require_control_heartbeat:=false
ros2 action send_goal /move_to core_interfaces/action/MoveTo '{task_id: dev, view_id: top}' --feedback
# /fetch, /stow, /fasten은 유효한 TargetPose와 계약의 ID를 채워 같은 방식으로 호출
# /locate 출력을 복사해 테스트 Goal의 pose를 구성할 수 있음

# 관제 단독 실행. Robot과 Vision은 별도 터미널에서 실행
ros2 run core_control control --ros-args --params-file core_bringup/config/workspace.yaml

# 비전 담당. 독립적인 Fake 검사 및 좌표 취득
ros2 run core_vision vision_node --ros-args -p scenario:=partial
ros2 service call /inspect core_interfaces/srv/Inspect '{task_id: dev, stage: INSPECT, view_id: top}'
ros2 service call /locate core_interfaces/srv/Locate '{task_id: dev, item_id: driver}'

# 언어 담당. 아직 연동되지 않은 서비스 계약 확인
ros2 run core_language llm_node
ros2 service call /parse_command core_interfaces/srv/ParseCommand '{text: check}'
ros2 run core_language voice_node
ros2 topic pub --once /announce core_interfaces/msg/Announcement '{task_id: dev, text: 안내}'
```

`camera.py`, `detector.py`, `transform.py`, `wakeword.py`, `stt.py`, `tts.py`는 호출 계약을 제시하는 템플릿입니다. 담당자는 장비·모델 선택 후 입력을 대체할 fixture를 사용해 개별 함수를 검증할 수 있습니다. Vision의 Fake 판정은 실제 RGB-D 프레임 없이 실행됩니다.

## 설계 적용과 영향

- 일관성을 위해 패키지명을 core_*로 변경했습니다. 외부 패키지 이름과 워크스페이스 이름은 유지했습니다.
- 설계에서 생략된 통신 필드를 정의했습니다. Task ID로 관제와 Episode를 연결하며 TargetPose에 유효 플래그와 ROS 표준 시각·좌표를 사용합니다.
- 누락 경로에서만 FETCH와 WAIT_WORKER, RECHECK를 통과합니다. 이 경로의 측면 검사는 RECHECK에서 수행합니다.
- 정지 시 REPORT를 발행 상태로 거치지 않고 작업 결과를 기록합니다. STOPPED 래치 유지와 결과 수집을 동시에 보장하기 위한 선택입니다.
- 브랜치 운영 및 JSONL·Episode 기록 시점은 구축 지시에 맞췄습니다. 실물 센서·힘·토크 검증은 남아 있습니다. CORE MONITOR의 Fake 검증은 아래에 별도로 기록합니다.

## 기존 P0-a Fake 회귀 검증

Ubuntu 24.04.5 / Jazzy / Cyclone DDS / Domain 50에서 `colcon build --symlink-install`로 7개 패키지를 빌드했습니다. 빌드·설치·로그는 기존 워크스페이스 산출물과 겹치지 않도록 임시 디렉터리에 두었습니다. Python 패키지 빌드에서 시스템 setuptools가 `pytest-repeat` egg 경고를 출력했습니다.

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`로 설치된 Launch를 기동하고 `tests/fake_integration.py`를 실행했습니다. 샌드박스에서 Cyclone DDS 인터페이스 열거가 차단되어 로컬 DDS 실행 권한으로 확인했습니다.

| 검증 | 확인한 결과 |
|---|---|
| 볼트 누락 | 9단계 순서 및 PASS |
| 반체결 | 마무리 레시피 및 PASS |
| 완전체결 | 확인 조임 레시피 및 PASS |
| 비전 판정 불가 | HOLD |
| 체결 실패 | FAIL 및 실패 Episode |
| 센서 불일치 | HOLD |
| 작업 중 정지 | WAIT_WORKER에서 STOPPED, 반복 check 거절, 명시적 resume으로 READY |
| DB 미실행 | PASS |
| LLM 미실행 | 실제 CLI로 check 후 PASS |
| 작업자 취소 | HOLD 및 READY |

TaskResult 토픽과 JSONL의 ID·판정, Event JSONL, Episode의 메타데이터·CSV·결과를 검사했습니다. 이 회귀 테스트의 검증 대상은 관제와 Fake 노드입니다. GUI는 별도 테스트에서 다룹니다. 다음 단계는 기존 두산 및 RG2 저수준 코드에서 장비별 어댑터를 분리하고 안전 조건과 교정을 검증하는 것입니다.

## 안정화 변경과 팀 재빌드

기존 Enum과 엔드포인트를 유지합니다. ROS 인터페이스 정의가 바뀌었으므로 기존 빌드의 노드와 새 메시지 생성물을 섞으면 안 됩니다.

| 인터페이스 | 이전 → 현재 | 담당 작업 |
|---|---|---|
| MoveTo·Fetch·Stow·Fasten | 로그 Feedback → progress 추가. 공통 Result에 stop_reason, Fasten에 tightening_valid 추가 | Robot 어댑터는 검증된 진행만 증가시키고 실측 유효성을 표시합니다. |
| ControlStatus | FSM·task_id → 관제 인스턴스, mode, 복구·정지, 판정, Vision 응답 상태, 허용 명령 및 최근 결과 ID | GUI·음성은 Transient Local로 받아 시각과 작업 ID를 확인합니다. |
| RobotStatus | busy/stopped/safe_to_move → 자세·파지·오류·활성 Action·힘/순응·정지 확인·마지막 종결 Goal ID | Robot 어댑터는 측정 유효성을 표시합니다. |
| TaskResult·LogEvent | 기본 결과·이벤트 → 출처, 정지 원인, 상단 판정, 레시피, 토크·회전수/유효성, 발행 노드 | 데이터 소비자는 JSONL/Episode의 추가 필드를 수용하고 Task ID와 Episode ID로 연결합니다. |
| JobCommand·EmergencyStop·ParseCommand | 기본 명령 → 출처, 래치 ID·만료·원인, 신뢰도 유효성과 확인 필요 여부 | GUI·CLI·음성과 별도 호출자는 새 생성물로 빌드합니다. |
| Inspect·Locate 및 영상 | 기존 타입·엔드포인트 유지 | Vision 팀은 기존 알고리즘과 Unicast 대상을 사용하고 Preview 토픽을 공유합니다. |

팀원은 같은 변경본을 받고 기존 오버레이가 source되지 않은 터미널에서 실행합니다.

```bash
cd ~/cobot2_ws
source /opt/ros/jazzy/setup.bash
colcon --log-base log/core-v01 build --base-paths src/core_project \
  --build-base build/core-v01 --install-base install/core-v01 --symlink-install
source install/core-v01/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=50
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

외부 Python 소비자는 새 오버레이를 source하고 컴파일된 소비자는 이어서 재빌드합니다. `require_control_heartbeat:=false`는 개별 Fake Action 시험에 사용합니다. 통합 실행 기본값은 true입니다.

## 운영 파라미터 및 기록

`core_bringup/config/workspace.yaml`에 Goal·취소·종결 응답, 개별 Action, Feedback 및 progress, Inspect·Locate, 관제 heartbeat 발행 주기, Robot heartbeat 기한, STOP·RESUME 응답, STOP 재시도 횟수, 작업자 완료 기한이 있습니다. `robot.yaml`에 Robot의 관제 heartbeat 감시 기한이 있습니다. 기본값은 Fake 개발용이며 실물 측정 후 조정합니다.

관제 JSONL과 Robot Episode 저장은 각각 제한된 Queue(1024건, 64건)의 작업 스레드에서 수행합니다. 정상 종료에서는 승인한 쓰기를 비우고, Queue 포화나 쓰기 실패는 ROS ERROR로 기록합니다. 토픽 결과가 파일보다 먼저 도착할 수 있습니다. 프로세스 강제 종료나 디스크 장애가 나면 대기 중인 기록이 손실될 수 있습니다.

## 검증 및 배포

```bash
ros2 launch core_bringup system.launch.py --show-args
ros2 launch core_bringup system.launch.py mode:=fake gui:=true
# 위 Launch 종료 후 실행
python3 tests/fake_integration.py
python3 tests/stability_integration.py
python3 tests/monitor_integration.py
python3 tests/video_pipeline.py
```

GUI 시험에는 로컬 그래픽 세션이 필요합니다. 영상 송수신과 실측 방법은 [영상 파이프라인](VIDEO_PIPELINE.md)을 참고합니다.

팀 검토 후 변경을 공유할 때 사용하는 명령입니다. 스테이징 전에 팀원 작업과 주소·데이터 파일을 확인합니다.

```bash
git status --short
git diff --check
git diff --stat
git add -p
# git add <검토한 신규 파일>
git diff --cached --stat
git commit -m "Stabilize core control and integrate monitor preview"
git push origin develop
```

## 변경 파일 목록

| 범위 | 파일 |
|---|---|
| Launch·설정 | `core_bringup/launch/system.launch.py`, `core_bringup/config/{workspace,robot}.yaml` |
| 공통 도구 | `core_common/core_common/{announcements,b1_camera_udp,reporter,robot_api,status_qos,types}.py` |
| 관제·GUI·CLI | `core_control/core_control/{control,control_interface,monitor_data,monitor_gui,task_assembly}.py`, `core_control/package.xml` |
| Action | `core_interfaces/action/{Fasten,Fetch,MoveTo,Stow}.action` |
| 메시지 | `core_interfaces/msg/{ControlStatus,LogEvent,RobotStatus,TaskResult}.msg` |
| 서비스 | `core_interfaces/srv/{EmergencyStop,JobCommand,ParseCommand}.srv` |
| 언어 계약 | `core_language/core_language/{llm_node,prompts,voice_commands}.py` |
| Robot·기록 | `core_robot/core_robot/{episode_recorder,motion,robot_node}.py` |
| 시험 | `tests/{fake_integration,monitor_integration,stability_integration,video_pipeline}.py` |
| 문서 | `README.md`, `docs/{DESIGN_V0.1,ARCHITECTURE,INTERFACES,SAFETY,DEVELOPMENT,ROADMAP,VIDEO_PIPELINE}.md` |

작업 시작 시 위 목록에는 팀원이 작업 중이던 안정화 코드와 UDP 모듈이 포함되어 있었습니다. 작업 범위는 워크트리 전체 변경이며 공유 전 스테이징 대상을 검토합니다.

## 안정화·영상 통합 재검증

Ubuntu 24.04, ROS 2 Jazzy 및 Cyclone DDS Domain 50의 LOCALHOST에서 실행했습니다. 빌드·설치·로그는 임시 오버레이에 두었습니다.

| 시험 | 결과 |
|---|---|
| colcon build | 7개 패키지 성공. 시스템 setuptools의 pytest-repeat egg 경고가 있었습니다. |
| Fake 통합 | 기존 9개 시나리오 및 취소 시험 총 10개 통과. 출처와 정지 원인의 JSONL 일치, Fasten 계측값과 Episode 값 일치도 확인했습니다. |
| 장애 주입 | 느린 성공, 실행 Timeout, Feedback Timeout, 진행 중단, Control 프로세스 강제 종료와 재시작, Control 없는 직접 STOP·중복 요청, STOP·RESUME 응답 유실, 제한된 재시도와 만료 RESUME, 지연된 Action 종결, 블로킹 Action 중 STOP, 최초 internal_error 보존, 공구 파지 중 RECOVER의 12개 확인을 통과했습니다. 시험 종료 시 일부 rclpy GuardCondition 해제 경고가 출력됐습니다. |
| GUI | 단독 실행, 다섯 버튼, 상태·로그·실측 표시, Raw 및 JPEG, NO SIGNAL·복구와 보조 OFF, Preview 비활성화, 재시작 시 마지막 결과, GUI 종료 후 작업 완료를 통과했습니다. 최종 로컬 시험에서 Raw 입력 중 STOP 클릭부터 관제·Robot 양쪽 래치 표시까지 0.134초를 관측했습니다. |
| UDP/Preview | GStreamer 기존 송수신 및 1·2개 루프백 목적지, Preview 활성화·비활성화 비교를 통과했습니다. [측정 수치](VIDEO_PIPELINE.md#로컬-합성-영상-결과) |

M0609·RG2 구동, 실제 정지 회로와 제어기 확인, D435i 캡처·인코딩, 현장 YOLO/검사 입력, 팀원 PC 두 대의 네트워크 부하·손실·지연은 미검증입니다.

## L/F 최소 실물 결과 통합

단일 볼트 Fake 조립 서비스와 Action을 유지하면서 외부 L/F 관측을 위한 두 읽기 전용 서비스를 추가했습니다. 두 위치를 개별 기록하기 위해 기존 ControlStatus와 TaskResult에 위치별 필드 및 출처를 추가했습니다. 배포 전 두 메시지를 사용하는 관제·GUI·Robot·음성·DB와 새 서비스 공급자는 동일한 `core_interfaces`를 재빌드해야 합니다. [담당자 구현 및 실행 계약](INTERFACES.md#lf-최소-통합-계약)

검증은 `python3 tests/inspection_integration.py` 또는 그래픽 세션에서 `python3 tests/inspection_integration.py --gui`로 수행합니다. 시험 공급자는 합성 응답으로 계약 분기를 검사합니다. 팀원 장비의 관측 및 알고리즘 검증은 별도입니다.

로컬 Cyclone DDS에서 7개 패키지 빌드, L/F 합성 공급자·GUI·JSONL 시험, Fake 공정 10개, 장애 주입 12개, 기존 CORE MONITOR 회귀를 통과했습니다. 장애 주입 종료 중 기존 rclpy 리소스 해제 경고가 발생했습니다. 실제 L/F 영상 및 힘·순응제어 알고리즘은 담당자 공급자를 연결한 현장 시험 대상입니다.
