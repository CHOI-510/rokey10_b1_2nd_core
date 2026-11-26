# CORE 영상 입력

카메라는 RealSense ROS 드라이버가 발행한 원본 `sensor_msgs/msg/Image`를 구독합니다. 기본 토픽과 관제의 기존 토픽·서비스·Action 이름은 `core_common/core_common/topics.py`에 있습니다.

| 소비자 | 기본 입력 | 이미 발행 중인 다른 토픽을 쓰는 경우 |
|---|---|---|
| 실물 체결 Action | `/camera/camera/color/image_raw` | Launch의 `camera_topic` |
| Vision 1 L/R 검사 | 체결과 동일한 원본 Image | 고정 카메라가 있다면 `vision_camera_topic` 및 해당 카메라의 `vision_roi_file` |
| GUI 손목 미리보기 | 체결과 동일한 토픽의 Raw Image | `wrist_image_topic`, 필요 시 `wrist_image_type:=compressed` |
| `slot_view`, `fasten_test` | `/camera/camera/color/image_raw` | CLI의 `--camera-topic` |
| Vision 1 ROI 보정 | `/camera/camera/color/image_raw` | CLI의 `--topic` |

`mode:=fake`의 GUI 미리보기 기본값은 OFF입니다. 영상 구독을 확인할 때는 `preview_enabled:=true`를 지정합니다. 손목 이외의 화면과 Wrench는 `side_image_topic`, `wrench_topic`을 지정한 경우에만 구독합니다.

같은 PC에서는 ROS 환경과 Domain을 맞춥니다. PC를 나눠 실행한다면 두 PC의 RMW·Domain과 DDS 검색 범위가 일치해야 합니다. `LOCALHOST` 범위에서는 다른 PC에 있는 발행기를 검색할 수 없습니다.

```bash
ros2 topic list -t
ros2 topic info /camera/camera/color/image_raw -v
ros2 run core_robot slot_view
```

`slot_view`를 위와 같이 인자 없이 실행하면 영상 진단만 합니다. 카메라 토픽 또는 해상도를 변경한 경우 검사 ROI와 체결 비전 보정이 실제로 사용하는 영상에 맞는지 확인합니다. GUI의 LIVE는 미리보기 수신 상태이며, 체결의 프레임 신선도와 보정 검사는 로봇 노드에서 확인합니다.
