# P0-a 개발 가이드

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
ros2 run core_robot robot_node --ros-args -p mode:=fake
ros2 action send_goal /move_to core_interfaces/action/MoveTo '{task_id: dev, view_id: top}' --feedback
# /fetch, /stow, /fasten은 유효한 TargetPose와 계약의 ID를 채워 같은 방식으로 호출
# /locate 출력을 복사해 테스트 Goal의 pose를 구성할 수 있음

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
- 브랜치 운영 및 JSONL·Episode 기록 시점은 구축 지시에 맞췄습니다. 실물 센서·힘·토크와 GUI 화면 검증은 후속 작업입니다.

## 검증 결과

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

TaskResult 토픽과 JSONL의 ID·판정, Event JSONL, Episode의 메타데이터·CSV·결과를 검사했습니다. 실물 통신, 카메라·음성·LLM 모델, GUI 화면은 이번 검증 범위 밖입니다. 다음 단계는 기존 두산 및 RG2 저수준 코드에서 장비별 어댑터를 분리하고 안전 조건과 교정을 검증하는 것입니다.
