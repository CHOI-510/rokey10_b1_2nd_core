# 실물 체결 백엔드 (`mode:=real`, `/fasten`)

실물 모드는 카메라의 L/R 존재 검사와 `core_robot`의 실물 `/fasten`을 실행합니다. Fake 모드는 모의 검사와 모의 Action만 실행합니다. 동작과 준비는 [L/R 실물 통합 사이클](#lr-실물-통합-사이클)을 참고합니다.

Doosan 브링업의 오버레이를 Core 오버레이보다 먼저 source합니다. RealSense 원본 Image와 Doosan 브링업을 같은 ROS Domain으로 실행합니다. 실제 영상 기본값은 `core_common.topics.CAMERA_COLOR`입니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/cobot2_ws/install/setup.bash

ros2 launch core_bringup system.launch.py mode:=real gui:=true
```

RealSense 원본 토픽이 기본값과 다르면 `camera_topic:=<토픽>`을 한 번 지정합니다. GUI 손목 미리보기, Vision 1, 체결은 해당 입력을 기본으로 사용합니다. 체결 구독은 depth 1이며 발행자 QoS를 확인해 구독 reliability를 결정합니다. 영상 미수신·오래된 시각·디코딩 실패는 첫 모션 전 `UNAVAILABLE`과 PREPARE 로그에서 확인할 수 있습니다.

### 체결 비전 보정

이 보정은 드라이버 위치·각도 및 체결 영상 ROI를 저장하는 작업이다. 체결에 사용할 카메라 입력과 해상도로 보정한다. 보정에 기록된 해상도가 현재 프레임과 일치해야 한다. 새 체결 보정에는 입력 토픽도 함께 기록한다. 이전 보정 파일은 해상도와 교정 설정을 검사하므로 당시의 동일 카메라·토픽인지 확인한다. Vision 1의 L/R 존재 검사 ROI는 별개로 설정한다. 보정 명령은 실제 로봇을 움직이므로 장비 준비와 현장 안전 절차를 마친 뒤 단독으로 실행한다.

```bash
ros2 run core_robot fasten_test --bolt 1 --calib-vision --skip-grip
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
| `point_id` | `fasten_point_map`(기본 `['L:2', 'R:1']`)으로 티칭 볼트 번호를 찾는다. 번호는 `core_robot.fastening.fasten_core.BOLT_POSES_TAUGHT`의 1(x 393.42, y 69.65) 또는 2(x 322.87, y 69.65)다. 208.4 mm TCP · 스펀지 50 mm 기준으로 잰 값이고, 드라이버 TCP와 `SPONGE_MM` 보정을 거친다. 없는 ID는 `INVALID`다. |
| `bolt_pose` | 실물 모드는 `valid=false`로 티칭 위치 사용을 명시한다. 티칭 좌표로 이동한 뒤 손목 카메라로 십자 중심의 x · y와 각도를 보정한다. |

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

- 위치별 실물 체결 제한은 `real_fasten_timeout`으로 설정하며 기본값은 180초다.
- progress는 최대 98에서 멈춘다. 약 196초를 넘기면 관제의 progress 감시(3초)에 걸린다.
- `confirm` 레시피를 실물에서 쓰려면 별도 확인 루틴이 필요하다.

## 실기에서 아직 확인하지 않은 것

- Doosan SDK와 CORE 로그의 준비·정지 서비스 이름이 현장 ROS 그래프와 일치하는지, `movel` 실행 중에도 정지 요청이 처리되는지
- `robot_node` 안에서 `DSR_ROBOT2` 전용 노드(`/dsr01/core_robot_dsr`)로 호출이 정상 동작하는지(실행기 밖에서 spin)
- 힘 제어 중 목표 힘을 5 N에서 12 N으로 바꿀 때 `DR_FC_MOD_REL` 기준이 유지되는지
- 취소 뒤 수직 후퇴와 정지 뒤 힘 해제가 실제 로봇에서 안전한지
- 비전 중심 맞추기의 자코비안과 이동 방향
- 실물 모드의 소프트웨어 준비 상태는 장비 인터록·공구 파지·촬영 자세를 확인하는 신호와 별개다. 취소 후 수직 후퇴한 경우 다음 검사 전에 촬영 자세를 다시 확인한다.

## 로봇 없이 하는 검증

```bash
python3 tests/real_fasten_contract.py
python3 tests/fasten_camera_input.py
```

- 가상 DSR 객체로 성공·미완료·토크 미측정·취소·정지·실패 반환값·지원하지 않는 목표를 확인한다. MoveStop 전송 및 재시도는 모의 응답으로 확인한다.
- 실물 모드 `robot_node`에 실제 `/fasten` Goal을 보내 거절 경로, 컨트롤러가 없을 때의 `UNAVAILABLE`, 중복 서버 차단을 확인한다.
- 원본 Image 시험은 가상 Doosan 객체로 실행한다. RGB→BGR, Action 수행 중 콜백, 무수신·오래된 프레임·디코딩 실패 시 모션 0건, 취소·정지 시 구독 해제, 늦게 도착한 이전 프레임 거절, 카메라 구독 없이 비전 시작 시 거절을 확인한다.

## L/R 실물 통합 사이클

`mode:=real`은 실물 관제 실행 경로입니다. `system.launch.py`가 Vision 1의 `/integration/inspect_positions`와 `core_robot`의 실물 `/fasten`을 같은 관제에 연결합니다. GUI의 L/R 카드에 존재, 체결 판정과 근거가 표시됩니다.

1. 작업 전에 드라이버를 파지하고 L=볼트 2, R=볼트 1의 티칭값과 TCP를 확인합니다. 이 사이클은 두 위치 모두 마무리 체결이 필요한 작업물을 대상으로 합니다. Vision 1은 볼트의 존재를 구분하며 기존의 체결 여부는 구분하지 못합니다.
2. Vision 1 보정에는 검사 시작 전과 체결 후 대기 자세에서 볼트 두 위치가 모두 보이는 영상이 필요합니다. 손목 영상으로 검사할 때에는 CHECK 전에 티칭된 대기 자세에 있어야 합니다. 고정 카메라를 별도로 쓰면 `vision_camera_topic`과 그 카메라용 `vision_roi_file`을 지정합니다. 체결용 손목 영상과 십자 홈 보정은 `camera_topic` 및 `fasten_test --calib-vision`을 사용합니다.
3. Doosan 브링업과 RealSense 후 [README의 실물 Launch](../README.md#lr-실물-체결-사이클)를 실행합니다. 기본 준비·정지 서비스는 설치된 SDK의 네임스페이스 및 서비스 접두사를 따릅니다. 정지 경로를 별도 지정하는 구성에는 `dsr_stop_service`를 사용합니다. 관제의 작업용 Action 서버는 하나만 실행하고, 독립 `fasten_test`를 종료한 상태에서 시작합니다.
4. CHECK → 실물 L/R 존재 검사입니다. 누락 위치가 있으면 WAIT_WORKER에서 준비 후 WORKER DONE으로 다시 검사합니다. 판정 불가, 영상 미수신, 검증되지 않은 검사 응답은 HOLD 처리합니다.
5. L/R이 모두 존재하면 티칭 위치 ID로 `finish` Action을 순서대로 실행합니다. 각 Action이 성공하고 토크·회전수·Episode ID를 확인해야 다음 위치로 갑니다. 마지막에는 대기 자세에서 신선한 영상으로 존재를 다시 검사합니다. PASS의 근거는 위치별 Action 토크 판정과 작업 후 영상입니다. 별도 체결 깊이 검사 센서는 이 경로에 포함되어 있지 않습니다. 위치별 Episode는 같은 `task_id`와 각 `point_id`로 기록합니다.
6. GUI STOP은 Doosan 소프트웨어 정지를 요청합니다. 관제 재요청 시 미응답 또는 거절된 요청을 다시 보내고, 컨트롤러가 접수한 요청은 중복 전송하지 않습니다. 이 경로의 `stop_confirmed`는 물리 정지 확인 신호가 연결될 때까지 표시하지 않으며, STOPPED에서 GUI RESUME으로 동작을 재개하지 않습니다. 첫 현장 시험에서 실제 정지 동작, 공구·카메라 자세, 좌우 티칭값과 힘·토크 판정을 확인합니다.

Jazzy에서 로봇 없이 새 관제 경로를 시험하려면 별도의 로컬 DDS에서 `python3 tests/real_cycle_integration.py`를 실행합니다. 이 시험은 합성 Vision·Fasten 공급자만 사용합니다.
