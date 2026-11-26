# Core Project v0.1

협동-2 볼트 조립 프로젝트의 ROS 2 관제 코어입니다. P0-a는 Fake Robot과 Fake Vision으로 전체 작업을 검증합니다.

## 설치 및 빌드

Ubuntu 24.04에서 ROS 2 Jazzy, Cyclone DDS RMW, colcon과 ROSIDL 빌드 도구가 필요합니다. GUI에는 선택적으로 `python3-tk`를 사용합니다. 설치 여부는 `ros2 pkg prefix rmw_cyclonedds_cpp` 등으로 확인합니다. 누락된 경우에만 설치합니다.

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rosidl-default-generators python3-colcon-common-extensions
```

빌드는 프로젝트 루트에서 실행합니다.

```bash
cd ~/cobot2_ws/src/core_project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Fake 실행

Launch, CLI, 테스트 터미널에서 다음 설정을 동일하게 적용합니다.

```bash
cd ~/cobot2_ws/src/core_project
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=50
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

```bash
ros2 launch core_bringup system.launch.py mode:=fake
```

다른 터미널에서 CLI로 작업을 제어합니다. 기본 시나리오 `missing`에서는 `WAIT_WORKER`에 도달한 뒤 `done`을 보냅니다.

```bash
ros2 topic echo /control_status
ros2 run core_control control_interface check
ros2 run core_control control_interface done
ros2 run core_control control_interface cancel
ros2 run core_control control_interface stop
ros2 run core_control control_interface resume
```

`resume`은 정지 확인 후 새 작업을 받을 수 있게 하는 명시적 명령입니다. `check`가 새 작업을 시작합니다. 결과는 `data/task_results.jsonl`에, 이벤트는 `data/event_logs.jsonl`에 기록됩니다. `data_dir:=/절대/경로`로 변경할 수 있습니다. ROS 토픽 기반 모니터는 `ros2 run core_control monitor_gui`로 실행합니다.

## 통합 테스트

기존 Core Launch를 종료한 후 실행합니다. 테스트는 각 시나리오를 독립적으로 기동하고 임시 경로에 기록합니다.

```bash
python3 tests/fake_integration.py
```

[설계 기준](docs/DESIGN_V0.1.md) · [아키텍처](docs/ARCHITECTURE.md) · [인터페이스](docs/INTERFACES.md) · [안전](docs/SAFETY.md) · [개발](docs/DEVELOPMENT.md) · [로드맵](docs/ROADMAP.md) · [Git](docs/GIT_GUIDE.md)
