# 독립 체결 시험

M0609 + RG2 가 쥔 일자 드라이버로 십자 홈 (장난감) 볼트를 조인다.
손목 카메라(OpenCV)로 십자 홈 **중심·각도**를 맞춘 뒤, 힘·순응 제어로 눌러 돌리고 J6 토크 + z 멈춤으로 완료를 판정한다.

## 파일

| 파일 | 역할 |
|---|---|
| `core_robot/core_robot/fastening/fasten_core.py` | 체결 루틴 · 상수 (CLI 와 `/fasten` Action 공용) — [튜닝] 값은 여기서 바꾼다 |
| `core_robot/core_robot/fastening/fasten_test.py` | 명령행 진입점 + 드라이버 건네기(그리퍼) |
| `core_robot/core_robot/fastening/slot_vision.py` | 십자 각도 · 중심 검출 (OpenCV, 가장 어두운 십자 덩어리 기준) |
| `core_robot/core_robot/fastening/udp_camera.py` | B-1 PC 가 보내는 RTP/JPEG UDP 영상 수신 |
| `core_robot/core_robot/fastening/slot_view.py` | 로봇 + 카메라로 검출만 실시간 확인 |

## 필요한 것

- 하드웨어 오버레이에 `doosan-robot2` (DSR_ROBOT2), `m0609_rg2_bringup`, `cobot2/robot_control` (그리퍼 `robot_control.onrobot` — `--skip-grip` 만 쓰면 불필요)
- `python3-opencv`, `python3-numpy`, `python3-gi` (GStreamer 1.0)
- 카메라 송신: B-1 PC 에서 `python3 -m core_common.camera_udp_h264 send --unicast-to <받는 PC IP>` (포트 6000)
  - 받는 PC 의 `camera_udp_h264.py view` 창은 닫는다 (같은 포트)

빌드는 [CORE 설치 절차](../README.md#설치-및-빌드)를 따릅니다. ROS 원본 영상은 [카메라 입력 예제](REAL_FASTEN.md)를 확인하세요.

## 실행 순서

터미널 1 — 브링업: `ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609`

```bash
# 1) 검출 확인 (선택): 볼트1 위로 이동 → p 로 ROI, ] 로 15° 돌려 각도가 따라오는지
ros2 run core_robot slot_view --bolt 1

# 2) 비전 보정 — 드라이버를 쥘 때마다 1회
#    날이 홈 가운데에 앉으면 Enter (i/k/j/l 로 0.5 mm 씩 옮기기, r 로 15° 돌리기) → ROI 클릭
ros2 run core_robot fasten_test --bolt 1 --calib-vision --skip-grip

# 3) 체결
ros2 run core_robot fasten_test --bolt 1 --skip-grip
```

옵션: `--no-vision` (비전 없이 홈 찾기 회전만), `--sim --no-vision` (에뮬레이터에서 순서만), `--skip-grip` (이미 드라이버를 쥐고 있음).

## 체결 로직 요약

1. 볼트 위 보기 높이(+40 mm) → **중심 맞추기** (십자 중심을 보정 때 축 픽셀로, 툴 x·y) → **각도 맞추기** (툴 rz)
2. +5 mm 에서 힘 제어(REL, 5 N)로 내려가 접촉 (|Fz−영점| ≥ 1.5 N 0.5 초, 또는 z 1 초 멈춤 + 0.8 N)
3. 12 N 으로 누르며 15° 씩 회전 → **완료**: |Δtau6| ≥ 1.5 Nm 이고 z 변화 < 0.5 mm 가 30° 연속
4. J6 한계 → 힘 해제 · 상승 · J6 90° 배수 되감기 → 1 로
5. 안전 중단: 드라이버 끝 z < 책상 + 5 mm, |Fz−영점| > 25 N. 종료 시 동작은 완료·취소·정지 사유에 따라 결정

## 반드시 다시 잴 값 (2026-11-26 이 셋업 기준)

`fasten_core.py` 맨 위:
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

## ROS 2 `/fasten` Action 으로 실행

`core_robot` 의 `robot_node` 를 `fasten_backend:=real` 로 띄우면 같은 루틴이 `/fasten` 으로 실행된다
(`--skip-grip` 과 같은 조건, `recipe_id` 는 `finish` 만). 매핑 · 결과 · 남은 확인 항목: [실물 체결 문서](REAL_FASTEN.md).
