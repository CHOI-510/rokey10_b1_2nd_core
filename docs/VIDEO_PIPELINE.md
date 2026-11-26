# 영상 공유와 CORE MONITOR

## 확인한 구성

| 경로 | 저장소에서 확인한 설정 |
|---|---|
| 카메라 소유 | 참조 워크스페이스 `~/ws_cobot_pjt/ws_dsr/src/rg2/m0609_rg2_bringup/launch/bringup_camera.launch.py`의 `realsense2_camera_node`. 참조 Launch는 색상·Depth, 정렬, 동기화를 설정합니다. Core Fake Launch는 카메라를 열지 않습니다. |
| ROS JPEG → UDP | `core_common/core_common/b1_camera_udp.py send`의 기본 입력은 `/camera/color/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`, JPEG)입니다. |
| UDP | GStreamer RTP/JPEG, payload 26, clock-rate 90000, MTU 1200, 기본 포트 6000. `--unicast-to`를 대상마다 한 번 지정합니다. 비전 PC 두 대의 기존 설정을 사용합니다. |
| UDP 수신 | 같은 모듈의 `view --unicast` 또는 Python `MulticastCamera(..., unicast=True)`. 지터 버퍼 기본 60 ms, appsink 최신 프레임 1개, OpenCV BGR 반환. 수신기가 ROS로 재발행하는 코드와 현장 대상 IP는 확인하지 못했습니다. |
| GUI | 기본 손목 입력은 `/camera/color/image_raw` (`sensor_msgs/msg/Image`), 보조 입력은 비어 있습니다. 명시 설정 시 기존 JPEG 토픽이나 YOLO 오버레이 토픽을 구독할 수 있습니다. |

위 토픽과 포트는 코드의 기본값입니다. 검사 장비에서 활성 카메라 노드, 실제 토픽, 색상·Depth 해상도, FPS, JPEG 품질 및 수신 포트는 해당 PC의 실행 설정으로 확인합니다.

```bash
ros2 node list
ros2 topic list -t
ros2 topic info <확인한_영상_토픽> -v
ros2 topic hz <확인한_영상_토픽>
ros2 topic bw <확인한_영상_토픽>
```

## 운영 방법

RealSense는 기존 드라이버 프로세스 한 개가 소유합니다. 비전 입력의 RGB-D, CameraInfo, Timestamp 정합과 기존 알고리즘 입력 품질을 유지합니다. GUI는 메인 서버나 이미 ROS 영상 토픽이 있는 PC에서 띄웁니다. 사용 가능한 ROS 토픽이 없다면 Preview를 끄고 관제를 사용할 수 있습니다. 기존 UDP 수신기가 ROS 토픽을 제공하는 배치에서는 그 토픽을 GUI에 지정합니다.

```bash
# 기존 송신 PC. 현재 업무에 사용하는 주소와 포트를 설정
python3 -m core_common.b1_camera_udp send --topic "$JPEG_TOPIC" --port "$PORT" \
  --unicast-to "$VISION_PC1" --unicast-to "$VISION_PC2" --stats

# 기존 비전 담당 PC
python3 -m core_common.b1_camera_udp view --unicast --port "$PORT" --headless

# 메인 서버 또는 영상 토픽이 이미 있는 PC
ros2 run core_control monitor_gui --ros-args \
  -p wrist_image_topic:="$JPEG_TOPIC" -p wrist_image_type:=compressed \
  -p preview_fps:=10.0 -p preview_width:=640 -p preview_height:=480

# Preview를 사용하지 않는 원격 관제 화면
ros2 run core_control monitor_gui --ros-args -p preview_enabled:=false
```

UDP 명령은 Python GI, GStreamer의 Gst/GstApp, jpegparse 및 RTP/UDP 플러그인과 수신용 OpenCV·NumPy가 필요합니다. GUI는 Pillow로 Raw 및 압축 영상을 처리합니다. 카메라 켜짐과 알고리즘 준비 상태는 별도 항목입니다. 비활성화하거나 토픽을 비우면 OFF, 유효한 프레임이 기본 2초간 없으면 NO SIGNAL입니다. LIVE 헤더에는 수신 FPS와 입력 해상도를 표시합니다. 디코딩과 축소는 GUI의 별도 작업 스레드에서 실행하며 디코딩 전후에는 입력별 최신 프레임만 유지합니다. Preview 크기와 디코딩 상한은 송신기 품질이나 DDS 전송량에 영향을 주지 않습니다. 센서 토픽과 검사 근거 자료가 이미 있으면 명시적으로 연결하고, 필요한 구간은 ROS 시각과 Episode ID로 연결해 rosbag2로 수집할 수 있습니다.

## 성능 조사

기존 송신기는 이미 압축된 JPEG를 구독하여 한 번 RTP 패킷화하고 `multiudpsink`로 복수 목적지로 보냅니다. 목적지 수에 따라 패킷 전송량이 늘어납니다. ROS 수신 queue depth 1과 GStreamer appsrc 최신 버퍼 1개를 사용합니다. Linux 네트워크·드라이버 부하, 카메라 캡처, 인코딩 지연 및 분산 수신 손실은 현장 실측이 필요합니다. `--stats`의 FPS는 송신기가 받은 JPEG 수이며 Mbps는 GStreamer RTP 바이트 카운터입니다. IP/UDP 및 링크 헤더는 포함되지 않습니다.

```bash
python3 tests/video_pipeline.py
```

비교 시 송신기 `--stats`, 각 수신기의 `view --headless`, 원본 토픽 FPS, CPU 사용률, NIC 송수신 카운터, GUI LIVE FPS를 같은 시간 구간에서 수집합니다. 서로 다른 PC의 타임스탬프를 비교할 때는 시간 동기화와 오차를 확인합니다. GUI의 프리뷰 신호가 끊겨도 정상 명령·STOP 경로는 계속 동작합니다. 실물 검사 입력의 유효성은 Vision 서비스가 책임지며, 유효한 결과가 없으면 관제가 작업을 보류합니다.

### 로컬 합성 영상 결과

640×360 JPEG(품질 70)를 15 FPS로 5초간 발행했습니다. Cyclone DDS는 Domain 50과 LOCALHOST를 사용했습니다. 첫 번째 목적지는 기존 UDP 수신기로 디코딩하고 두 번째 목적지는 UDP 페이로드 수신량을 측정했습니다. Preview는 ROS 수신·디코딩 스레드의 출력 FPS입니다.

| UDP 목적지 | ROS Preview | 첫 수신 디코딩 | Preview 디코딩 | 송신기 RTP 페이로드 합계 | 두 번째 목적지 페이로드 | 수신·발행·Preview 시험 프로세스 CPU |
|---|---|---|---|---|---|---|
| 1 | OFF | 15.0 FPS | 0 | 9.22 Mbps | — | 코어 한 개 기준 21.3% |
| 2 | OFF | 15.0 FPS | 0 | 18.43 Mbps | 9.19 Mbps | 15.3% |
| 2 | ON | 15.0 FPS | 9.6 FPS | 18.43 Mbps | 9.18 Mbps | 26.3% |

행별 CPU는 짧은 단일 측정이며 카메라와 인코더를 포함하지 않습니다. 송신기 합계는 안정 구간의 GStreamer 카운터입니다. 두 번째 목적지는 디코딩 FPS를 측정하지 않았습니다. 실제 캡처·인코딩, 현장 NIC 및 DDS 트래픽, 두 비전 PC의 프레임 유실·지연, 장시간 부하는 미검증입니다.
