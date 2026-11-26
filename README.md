# rokey10_b1_2nd_core

협동-2 볼트 조립 프로젝트의 ROS 2 관제 코어입니다. 현재 구현은 Fake Robot과 Fake Vision으로 단일 작업 FSM, 검사·체결 판정, 정지·복구, JSONL 및 Episode 기록을 실행합니다. Fake와 실물 모드는 동일한 L/R 관제 FSM과 ROS 메시지로 **CORE MONITOR**의 상태와 결과를 갱신합니다. 기본 실행에는 DB와 외부 AI API가 필요 없습니다. 외부 결과를 연결하는 팀원은 [L/R 검사 전용 실행](#lr-외부-검사-결과-통합)을 사용합니다.

실물 단일 사이클은 준비된 볼트 두 개, Doosan 체결 어댑터와 Vision 1 카메라를 사용합니다. 별도 체결 판정, 공구 자동 인계·거치, 음성 및 현장 검증은 로드맵을 참고하세요. [로드맵](docs/ROADMAP.md)

## 패키지와 진입점

| 패키지 | 담당 |
|---|---|
| `core_bringup` | 시스템 Launch와 파라미터 |
| `core_common` | 공통 타입·오류·기록·ROS 이름 정의 |
| `core_control` | FSM, CLI, CORE MONITOR |
| `core_interfaces` | ROS Action·Service·Message |
| `core_language` | 음성·LLM 연동 계약 |
| `core_robot` | 로봇 Action, 체결 구현과 단독 진단 명령 |
| `core_vision` | Fake 검사와 L/R 존재 검사 |

실행 모드는 Fake L/R 체결, 실물 L/R 체결, 외부 L/R 검사입니다. 실물 체결 시험의 공용 구현과 `fasten_test`·`slot_view` 진입점은 `core_robot`에 있습니다. 장비 연결과 두 실행 경로의 적용 범위는 [실물 체결 문서](docs/REAL_FASTEN.md)를 참고하세요.

## 설치 및 빌드

Ubuntu 24.04에서 ROS 2 Jazzy와 Cyclone DDS를 사용합니다. Jazzy apt 저장소가 설정된 환경에서 필요한 패키지를 설치합니다. GUI는 Tkinter와 Pillow를 사용하며 그래픽 디스플레이가 필요합니다.

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rosidl-default-generators ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-cv-bridge \
  python3-colcon-common-extensions python3-tk python3-pil python3-numpy python3-opencv
mkdir -p ~/cobot2_ws/src
git clone -b main https://github.com/CHOI-510/rokey10_b1_2nd_core.git ~/cobot2_ws/src/core_project
```

이미 체크아웃했다면 clone은 생략합니다. 새 터미널에서 워크스페이스 루트로 이동해 빌드합니다. Doosan 오버레이를 먼저 source하면 `src/cobot2`의 그리퍼 패키지(`robot_control`)도 함께 빌드됩니다.

```bash
cd ~/cobot2_ws
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
colcon build --symlink-install
```

Launch, GUI, CLI와 테스트를 실행하는 각 터미널에서 같은 오버레이를 source합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/cobot2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=50
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

위 설정은 같은 PC 안에서만 ROS 노드를 검색합니다. 팀원 PC에 GUI나 CLI를 따로 띄울 때는 각 PC에서 같은 버전의 인터페이스를 빌드·source하고, 두 PC의 RMW와 팀에서 합의한 Domain ID를 맞춥니다. DDS 검색과 트래픽이 허용된 같은 서브넷에서 양쪽의 검색 범위를 `SUBNET`으로 설정하고 노드를 시작합니다. 이 문서의 회귀 검증은 한 PC의 `LOCALHOST` 범위에서 수행했습니다.

## Fake 관제와 CLI

```bash
ros2 launch core_bringup system.launch.py mode:=fake
```

기본 시나리오는 `missing`입니다. Launch는 관제, Fake Robot, Fake Vision, CORE MONITOR를 띄웁니다. 화면 없이 시험할 때는 `gui:=false`를 지정합니다. Fake와 실물은 `/integration/inspect_positions`와 `/fasten`을 동일하게 호출하며, Fake 공급자가 검사 결과와 체결 Action 결과를 생성합니다. 관제는 서비스 검색, 응답, Action 결과를 기다립니다.

| `scenario` | Fake 공급자 응답 |
|---|---|
| `missing` | 첫 검사 L 누락, R 존재. WORKER DONE 후 둘 다 존재 |
| `partial`, `seated` | 두 위치 존재. 이전 시나리오 이름은 초기 존재 판정만 지정하며 같은 L/R 체결 경로로 진행 |
| `vision_unknown` | L 판정 불가로 HOLD |
| `fasten_failure` | L 성공, R 미체결로 FAIL |
| `sensor_mismatch` | 체결 후 확인에서 R 누락으로 HOLD |

다른 터미널에서 상태를 보거나 아래 CLI 명령 중 필요한 명령을 실행합니다.

```bash
ros2 topic echo /control_status --qos-durability transient_local
ros2 run core_control control_interface check
ros2 run core_control control_interface done
ros2 run core_control control_interface cancel
ros2 run core_control control_interface stop
ros2 run core_control control_interface resume
```

기본 기록은 실행 디렉터리 아래의 `data/task_results.jsonl`, `data/event_logs.jsonl`, `data/episodes/`에 저장합니다. Launch의 `data_dir:=/절대/경로`로 변경할 수 있습니다.

## CORE MONITOR

다음 두 방식 중 하나를 사용합니다.

```bash
# 이미 실행 중인 Fake 관제에 GUI 연결
ros2 run core_control monitor_gui --ros-args -p mode:=fake

# Fake 관제·Fake 노드와 GUI를 한 Launch에서 실행
ros2 launch core_bringup system.launch.py mode:=fake gui:=true
```

단독 명령은 GUI 프로세스만 띄웁니다. 관제와 Fake Robot·Fake Vision을 별도로 실행해야 작업 상태를 받고 명령으로 Fake 작업을 수행할 수 있습니다. GUI를 닫아도 관제는 계속 실행됩니다.

| 버튼 | CLI | 동작 |
|---|---|---|
| CHECK | `check` | READY에서 이동 허가를 확인한 뒤 새 작업을 시작합니다. |
| WORKER DONE | `done` | WAIT_WORKER의 현재 작업을 이어갑니다. 재검사를 실행하고 L/R 모두 존재해야 체결로 진행합니다. |
| RESUME | `resume` | STOPPED 또는 RECOVER에서 물리 조건을 확인한 뒤 명시적으로 복귀를 요청합니다. READY가 표시되면 CHECK로 새 작업을 시작할 수 있습니다. |
| CANCEL | `cancel` | 진행 중인 작업의 취소를 요청합니다. |
| STOP | `stop` | GUI는 관제와 Robot에 독립적으로 소프트웨어 정지를 요청합니다. CLI는 관제 경로를 호출합니다. 물리적 비상정지 장치가 필요합니다. |

### 볼트 누락 예제

1. 위 통합 Launch를 실행하고 CORE MONITOR에 READY가 표시될 때까지 기다립니다.
2. CHECK를 누르면 검사 서비스 응답 뒤 WAIT_WORKER에 도달합니다. GUI는 L 누락, R 존재를 표시합니다.
3. WORKER DONE을 누릅니다. 재검사와 체결을 마치면 화면에 PASS가 표시되고 관제는 READY로 돌아갑니다.
4. 결과 카드와 이벤트 로그에서 L/R별 존재·체결과 `FAKE / FAKE_ACTION_RESULT` 근거를 확인합니다. 검사·체결 공급자가 없거나 응답 기한을 넘기면 HOLD가 표시됩니다.

Fake 실행은 GUI 영상 구독을 기본으로 끕니다. 미리보기를 시험할 때는 `preview_enabled:=true`를 사용합니다. 실물에서는 손목 미리보기, Vision 1과 체결이 `core_common.topics.CAMERA_COLOR`의 원본 영상을 기본으로 구독합니다.

기본 영상은 `/camera/camera/color/image_raw`입니다. 다른 이름으로 발행 중이면 Launch에 `camera_topic:=<토픽>`, 카메라를 쓰는 단독 명령에는 `--ros-args -p camera_topic:=<토픽>`을 사용합니다. 선택 입력과 DDS 확인 방법은 [영상 입력](docs/VIDEO_PIPELINE.md)을 참고하세요. 공통 인터페이스가 바뀌었으므로 관제·Robot·GUI·언어 및 외부 소비자는 같은 변경본을 빌드해야 합니다. [팀 재빌드](docs/DEVELOPMENT.md#안정화-변경과-팀-재빌드)

## 통합 테스트

같은 Domain에서 실행 중인 Core Launch를 종료한 뒤 저장소 루트에서 실행합니다. 테스트는 Fake Launch를 자체 기동하고 임시 디렉터리에 기록합니다.

```bash
python3 tests/fake_integration.py
python3 tests/stability_integration.py
# 그래픽 디스플레이가 있는 터미널에서 실행
python3 tests/monitor_integration.py
```

[설계 기준](docs/DESIGN_V0.1.md) · [아키텍처](docs/ARCHITECTURE.md) · [인터페이스](docs/INTERFACES.md) · [안전](docs/SAFETY.md) · [개발·검증](docs/DEVELOPMENT.md) · [로드맵](docs/ROADMAP.md) · [Git](docs/GIT_GUIDE.md)

## L/R 외부 검사 결과 통합

`ros2 launch core_bringup system.launch.py mode:=inspection gui:=true`로 외부 검사 결과를 받습니다. CHECK로 L/R 존재를 확인하고 WORKER DONE 뒤 재검사합니다. 두 위치가 모두 PRESENT이면 외부 체결 상태 검사 결과를 표시·기록합니다. 이 모드는 외부 결과를 읽으며 Robot Node와 자동 조립 동작은 실행 범위에 포함되지 않습니다. 팀원 서비스 계약, 음성 명령 예시 및 DB 구독은 [L/R 통합 계약](docs/INTERFACES.md#lr-최소-통합-계약)을 참고하세요.

## L/R 실물 체결 사이클

Doosan 브링업과 RealSense를 실행하고, 같은 ROS Domain에서 CORE를 시작합니다.

```bash
ros2 launch core_bringup system.launch.py mode:=real gui:=true
```

이 실행은 관제, Vision 1, 실물 `/fasten` 서버, GUI를 함께 띄웁니다. GUI의 `CHECK`가 실물 L/R 존재 검사를 시작합니다. 준비한 볼트가 빠져 있으면 `WORKER DONE`을 기다린 뒤 다시 검사합니다. 실물 체결 L→R이 각각 계측 성공을 반환하고, 작업 후 신선한 영상에서 두 볼트가 확인되면 PASS입니다.

대상은 **미체결 상태로 준비한 볼트 두 개**입니다. 드라이버를 미리 파지하고, 볼트 위치·TCP, Vision 1 ROI, 체결용 비전 보정을 준비합니다. 시작 자세와 카메라·실물 STOP의 확인 항목은 [실물 통합 절차](docs/REAL_FASTEN.md#lr-실물-통합-사이클)를 참고하세요.
