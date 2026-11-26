# cobot_robot — 협동2 1차 코어테스트 나사 체결

M0609 + RG2 가 쥔 일자 드라이버로 십자 홈 (장난감) 볼트를 조인다.
손목 카메라(OpenCV)로 십자 홈 **중심·각도**를 맞춘 뒤, 힘·순응 제어로 눌러 돌리고 J6 토크 + z 멈춤으로 완료를 판정한다.

## 파일

| 파일 | 역할 |
|---|---|
| `cobot_robot/fasten_test.py` | 보정 · 체결 본체 (상수는 파일 맨 위) |
| `cobot_robot/slot_vision.py` | 십자 각도 · 중심 검출 (OpenCV, 가장 어두운 십자 덩어리 기준) |
| `cobot_robot/udp_camera.py` | B-1 PC 가 보내는 RTP/JPEG UDP 영상 수신 |
| `cobot_robot/slot_view.py` | 로봇 + 카메라로 검출만 실시간 확인 |

## 필요한 것

- 같은 작업공간에 `doosan-robot2` (DSR_ROBOT2), `m0609_rg2_bringup`, `cobot2/robot_control` (그리퍼 `robot_control.onrobot` — `--skip-grip` 만 쓰면 불필요)
- `python3-opencv`, `python3-numpy`, `python3-gi` (GStreamer 1.0)
- 카메라 송신: B-1 PC 에서 `python3 b1_camera_udp.py send --unicast-to <받는 PC IP>` (포트 6000)
  - 받는 PC 의 `b1_camera_udp.py view` 창은 닫는다 (같은 포트)

## 빌드

```bash
cd ~/cobot2_ws/src && unzip cobot_robot_*.zip
cd ~/cobot2_ws && colcon build --packages-select cobot_robot && source install/setup.bash
```

## 실행 순서

터미널 1 — 브링업: `ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609`

```bash
# 1) 검출 확인 (선택): 볼트1 위로 이동 → p 로 ROI, ] 로 15° 돌려 각도가 따라오는지
ros2 run cobot_robot slot_view --bolt 1

# 2) 비전 보정 — 드라이버를 쥘 때마다 1회
#    날이 홈 가운데에 앉으면 Enter (i/k/j/l 로 0.5 mm 씩 옮기기, r 로 15° 돌리기) → ROI 클릭
ros2 run cobot_robot fasten_test --bolt 1 --calib-vision --skip-grip

# 3) 체결
ros2 run cobot_robot fasten_test --bolt 1 --skip-grip
```

옵션: `--no-vision` (비전 없이 홈 찾기 회전만), `--sim --no-vision` (에뮬레이터에서 순서만), `--skip-grip` (이미 드라이버를 쥐고 있음).

## 체결 로직 요약

1. 볼트 위 보기 높이(+40 mm) → **중심 맞추기** (십자 중심을 보정 때 축 픽셀로, 툴 x·y) → **각도 맞추기** (툴 rz)
2. +5 mm 에서 힘 제어(REL, 5 N)로 내려가 접촉 (|Fz−영점| ≥ 1.5 N 0.5 초, 또는 z 1 초 멈춤 + 0.8 N)
3. 12 N 으로 누르며 15° 씩 회전 → **완료**: |Δtau6| ≥ 1.5 Nm 이고 z 변화 < 0.5 mm 가 30° 연속
4. J6 한계 → 힘 해제 · 상승 · J6 90° 배수 되감기 → 1 로
5. 안전 중단: 드라이버 끝 z < 책상 + 5 mm, |Fz−영점| > 25 N. 끝나면 항상 대기 자세 복귀

## 반드시 다시 잴 값 (2026-11-26 이 셋업 기준)

`fasten_test.py` 맨 위:
- `BOLT_POSES_TAUGHT` — 볼트 좌표 (208.4 TCP · 스펀지 50 mm 위에서 티칭)
- `DRIVER_TCP_MM = 369.8` — 플랜지 → 드라이버 끝
- `DESK_Z = -27.1` — 책상 높이 (base 기준)
- `SPONGE_MM = 0.0` — 지금 받침 스펀지 두께
- `TOOL_WEIGHT_NAME` — 컨트롤러에 등록된 툴 무게 이름 (지금 `Tool Weight_1` 은 실패 → Fz 영점 빼기로 우회)
- `[튜닝]` 표시: 누르는 힘, 완료 토크, 중심·각도 허용치 등

비전 보정 파일 `~/cobot2_ws/logs/vision_calib.json` 은 드라이버를 쥔 각도에 따라 달라지므로 공유하지 않는다.

## 로그

`~/cobot2_ws/logs/` — `fasten_*.csv` (측정마다 한 줄: 자세 · J6 · Fz · tau6 · 회전량), 비전 측정 사진 `*_vNN.jpg` / `*_vNN_raw.jpg`.

## 실기 미확인

- 힘 제어 중 목표 힘을 5 → 12 N 으로 바꿀 때 REL 기준 유지 여부
- 중심 맞추기 자코비안 · 이동 방향의 실제 동작
