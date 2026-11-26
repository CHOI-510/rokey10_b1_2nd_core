# 실물 체결 백엔드 (`/fasten`, `fasten_backend:=real`)

`core_robot`의 체결 루틴(`core_robot/core_robot/fastening/fasten_core.py`)을 기존 `/fasten` Action으로 호출한다. `Fasten.action`은 바꾸지 않았다. MoveTo · Fetch · Stow는 그대로 Fake다.

`mode:=fake fasten_backend:=real`에서 단일 볼트 FSM의 체결만 실물입니다. MoveTo·Fetch·Stow 및 측면 검사는 Fake이고, L/R 조회에 쓰는 `mode:=inspection`은 Robot Action을 호출하지 않습니다. 실물 정지 확인 및 복구 조건에 대한 현장 검증이 남아 있습니다. 독립 실험의 명령과 튜닝 기록은 [독립 체결 시험](FASTEN_BENCH.md)을 참고하세요.

## 전제

- 로봇은 작업 시작 전에 이미 드라이버를 쥐고 있고, 체결 후에도 계속 쥔다. Action 경로는 `ros2 run core_robot fasten_test --skip-grip`과 같은 순서로 실행한다. 드라이버 인계와 그리퍼 개폐는 하지 않는다.
  - 순서: 목표 검사 → Doosan 컨트롤러 확인 → 카메라 · 비전 보정 확인 → TCP 설정(`setup_tool`) → `READY_J` → 체결 → `READY_J`
- Doosan 브링업(`m0609_rg2_bringup`)이 같은 Domain에서 실행 중이어야 한다. 카메라 입력은 ROS 원본 Image 또는 기존 B-1 PC UDP 송신(포트 6000)을 사용한다. 두 경로 모두 `~/cobot2_ws/logs/vision_calib.json`을 사용한다. 비전 보정은 드라이버를 쥘 때마다 `fasten_test --calib-vision`으로 다시 만든다.
- `dsr_msgs2`와 `DSR_ROBOT2`가 있는 Doosan 워크스페이스를 Core 오버레이보다 먼저 source한다.

## 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash                # Doosan · robot_control
source ~/cobot2_ws/install/core-v01/local_setup.bash # Core
ros2 launch core_bringup system.launch.py mode:=fake fasten_backend:=real
```

### ms-05의 USB RealSense 원본 토픽 사용

Jazzy의 `cv_bridge`와 `sensor_msgs`를 설치하고 기존 Core 빌드 절차로 `core_robot`, `core_bringup`을 포함한 오버레이를 다시 빌드한다. RealSense 드라이버와 Doosan 브링업을 기동한 환경에서 두 터미널 모두 같은 DDS 설정과 아래 오버레이를 사용한다. Doosan 설치 경로가 다른 PC에서는 해당 워크스페이스를 먼저 source한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/cobot2_ws/install/core-v01/local_setup.bash

ros2 topic info /camera/camera/color/image_raw -v
ros2 launch core_bringup system.launch.py \
  mode:=fake fasten_backend:=real gui:=true \
  wrist_image_topic:=/camera/camera/color/image_raw \
  fasten_camera_topic:=/camera/camera/color/image_raw
```

`fasten_camera_topic`은 체결 Action용 원본 Image 입력, `wrist_image_topic`은 GUI 미리보기 입력이다. 체결 구독은 depth 1로 유지한다. 발견된 발행자가 모두 Reliable이면 Reliable을 선택하며 그 외에는 Best Effort를 사용한다. 영상은 `cv_bridge`로 BGR로 변환한다. ROS 입력을 선택하면 UDP 송신·포트 6000·JPEG 토픽이 필요 없다. 인자를 생략하거나 비우면 기존 UDP 선택을 사용한다. 영상 미수신·오래된 시각·디코딩 실패는 첫 모션 전 `UNAVAILABLE`과 PREPARE 로그로 확인할 수 있다. 정렬 중에도 동작 후 촬영한 프레임만 읽으며, 입력 오류는 체결 중단으로 이어진다.

### 체결 비전 보정

이 보정은 드라이버 위치·각도 및 체결 영상 ROI를 저장하는 작업이다. 체결에 사용할 카메라 입력과 해상도로 보정한다. 보정에 기록된 해상도가 현재 프레임과 일치해야 한다. 이전 보정 파일에 해상도 기록이 없다면 새 보정을 만든다. Vision 1의 L/R 존재 검사 ROI는 별개로 설정한다. 보정 명령은 실제 로봇을 움직이므로 장비 준비와 현장 안전 절차를 마친 뒤 단독으로 실행한다.

```bash
# USB RealSense의 ROS 원본 영상
ros2 run core_robot fasten_test --bolt 1 --calib-vision \
  --camera-topic /camera/camera/color/image_raw

# 기존 UDP 송신을 쓰는 경우
ros2 run core_robot fasten_test --bolt 1 --calib-vision
```

같은 ROS 그래프에서 `/fasten` 서버는 하나만 허용한다.

- 실물 `robot_node`는 시작할 때 이미 `/fasten` 서버가 있으면 오류로 끝난다.
- 실행 중에도 다른 `/fasten` 서버가 보이면 Goal을 거절해서 로봇을 움직이지 않는다.
- Fake는 기존 시작 동작을 유지한다. 인-프로세스 테스트가 노드를 다시 만들 때 그래프 목록이 늦게 갱신되기 때문이다.
- 서버가 둘이면 관제의 Action 클라이언트가 어느 쪽 응답이든 받을 수 있다. 로봇 없이 시험해 보면 실물 서버가 모르는 Goal에 빈 결과(`success=false`, `error_code=0`, `state=""`)를 돌려준다. 실물 쪽 거절은 로봇을 움직이지 않게 하는 안전장치일 뿐이고, 관제 결과를 바로잡아 주지는 않는다. **robot_node는 하나만 띄운다.**

## Goal 매핑 — 모션 전에 검사한다

| 필드 | 실물 동작 |
|---|---|
| `recipe_id` | `finish`만 지원한다. `confirm`은 `NOT_IMPLEMENTED`(8)로 거절하고 `finish`로 바꾸지 않는다. 그 밖의 값은 `INVALID`(1)다. |
| `point_id` | `fasten_point_map`(기본 `['assembly_bolt:1']`)으로 티칭 볼트 번호를 찾는다. 번호는 `core_robot.fastening.fasten_core.BOLT_POSES_TAUGHT`의 1(x 393.42, y 69.65) 또는 2(x 322.87, y 69.65)다. 208.4 mm TCP · 스펀지 50 mm 기준으로 잰 값이고, 드라이버 TCP와 `SPONGE_MM` 보정을 거친다. 없는 ID는 `INVALID`다. |
| `bolt_pose` | Fake와 같은 유효성 검사(`valid_pose`)만 한다. **모션에는 쓰지 않는다.** 티칭 좌표로 이동한 뒤 손목 카메라로 십자 중심의 x · y와 각도를 보정한다. |

## Result

| 상황 | success / error_code | state | tightening_valid |
|---|---|---|---|
| 토크 완료(\|Δtau6\| ≥ 1.5 N·m, z 정지가 30° 연속) | true / OK | SEATED | true |
| 최대 회전 · 사이클 도달 | false / FAILED | UNKNOWN | false |
| 안전 중단(z 하한, \|Fz\| > 25 N), 예외 | false / FAILED | UNKNOWN | false |
| 토크를 한 번도 읽지 못함 | false / FAILED | UNKNOWN | false |
| 컨트롤러 · 카메라 · 보정 없음 | false / UNAVAILABLE (모션 없음) | UNKNOWN | false |
| 취소 | false / CANCELLED. 힘을 해제하고 보기 높이까지만 수직 후퇴한다. | UNKNOWN | false |
| 정지, 이동 허가 상실, 관제 heartbeat 상실 | false / STOPPED. Doosan `move_stop`(DR_QSTOP)을 요청하고, 힘을 해제하며, 더 움직이지 않는다. | UNKNOWN | false |

- `peak_torque`: 조이는 동안 \|tau6 − 조이기 직전 tau6\|의 최댓값(N·m)이다. 한 번도 읽지 못하면 0.0이다.
- `turns`: 홈 찾기와 조이기에서 돈 툴 회전량 ÷ 360이다.
- `stop_reason`은 기존처럼 정지일 때만 채운다. 그 밖의 사유는 피드백 `log_msg`, 로그, episode `result.json`의 `detail`에 남는다. 체결 CSV 경로는 `fasten_log`에 남는다.
- 피드백은 1초마다 보낸다. progress는 2초마다 1씩 올라 최대 98이다.

## 관제 쪽에서 맞춰야 할 것 (이번 변경에서 손대지 않음)

- **`fasten_timeout` 기본값이 30초다.** 실물은 접촉 하강만 최대 60초이고, 체결 전체는 수 분이 걸릴 수 있다. 실물 운용 전에 늘려야 한다(예: 300초).
- progress는 최대 98에서 멈춘다. 약 196초를 넘기면 관제의 progress 감시(3초)에 걸린다.
- `confirm` 레시피를 실물에서 쓰려면 별도 확인 루틴이 필요하다.
- FSM의 Stow · Fetch 제거와 측면 재검사 연동은 관제 담당이 맡는다.

## 실기에서 아직 확인하지 않은 것

- `/dsr01/dsr_controller2/motion/move_stop` 서비스 이름이 맞는지, `movel` 실행 중에도 정지 요청이 처리되는지
- `robot_node` 안에서 `DSR_ROBOT2` 전용 노드(`/dsr01/core_robot_dsr`)로 호출이 정상 동작하는지(실행기 밖에서 spin)
- 힘 제어 중 목표 힘을 5 N에서 12 N으로 바꿀 때 `DR_FC_MOD_REL` 기준이 유지되는지
- 취소 뒤 수직 후퇴와 정지 뒤 힘 해제가 실제 로봇에서 안전한지
- 비전 중심 맞추기의 자코비안과 이동 방향
- 이동 허가(`fake_motion_clear`)는 아직 소프트웨어 파라미터다. 실물 안전 입력과 연결되지 않았다.

## 로봇 없이 하는 검증

```bash
python3 tests/real_fasten_contract.py
python3 tests/fasten_camera_input.py
```

- 가상 DSR 객체로 성공 · 미완료 · 토크 미측정 · 취소 · 정지 · 지원하지 않는 목표의 결과를 확인한다.
- 실물 모드 `robot_node`에 실제 `/fasten` Goal을 보내 거절 경로, 컨트롤러가 없을 때의 `UNAVAILABLE`, 중복 서버 차단을 확인한다.
- 원본 Image 시험은 가상 Doosan 객체로 실행한다. RGB→BGR, Action 수행 중 콜백, 무수신·오래된 프레임·디코딩 실패 시 모션 0건, 취소·정지 시 구독 해제, 늦게 도착한 이전 프레임 거절, 빈 파라미터의 UDP 선택을 확인한다.
