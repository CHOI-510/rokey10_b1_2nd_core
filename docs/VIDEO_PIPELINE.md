# CORE 영상 입력

메인 카메라 이름은 `camera_topic` 한 곳에서 지정합니다. 이름을 생략하면 `core_common/core_common/topics.py`의 `CAMERA_COLOR` 값인 `/camera/camera/color/image_raw`를 구독합니다. RealSense 드라이버가 이미 발행 중인 이름은 `ros2 topic list -t`로 확인할 수 있습니다.

## 통합 Launch

```bash
CAMERA_TOPIC=/camera/camera/color/image_raw
ros2 launch core_bringup system.launch.py mode:=real gui:=true camera_topic:="$CAMERA_TOPIC"
```

실물 Launch는 원본 `sensor_msgs/msg/Image`를 쓰는 Vision 1, 체결, GUI 손목 미리보기에 같은 값을 전달합니다. Fake에서 미리보기를 확인하려면 `preview_enabled:=true`를 함께 지정합니다.

별도 영상이 실제로 발행되는 경우에는 해당 역할의 입력만 덮어씁니다. `vision_camera_topic`은 고정 카메라로 L/R을 검사할 때 사용하고 해당 카메라의 `vision_roi_file`과 짝을 맞춥니다. `side_image_topic`은 GUI에 두 번째 영상을 표시할 때 사용합니다. `wrist_image_type`과 `side_image_type`은 영상 메시지 형식입니다. GUI 단독 실행에서는 `wrist_image_type:=compressed`로 압축 영상을 표시할 수 있습니다.

## 카메라를 사용하는 단독 실행

```bash
ros2 run core_robot slot_view --ros-args -p camera_topic:="$CAMERA_TOPIC"
ros2 run core_robot fasten_test --bolt 1 --skip-grip --ros-args -p camera_topic:="$CAMERA_TOPIC"
ros2 run core_vision vision1_calibrate --ros-args -p camera_topic:="$CAMERA_TOPIC"
ros2 run core_vision vision1_node --ros-args -p real_camera:=true -p camera_topic:="$CAMERA_TOPIC"
ros2 run core_control monitor_gui --ros-args -p mode:=real -p camera_topic:="$CAMERA_TOPIC"
ros2 run core_robot robot_node --ros-args -p mode:=real -p camera_topic:="$CAMERA_TOPIC"
```

통합 Launch를 실행할 때에는 그 안의 Vision 1, GUI, 로봇 노드가 기동합니다. 위 단독 명령은 각 구성 요소를 개별 실행할 때 사용합니다. `slot_view`는 `--bolt` 생략 시 영상 진단만 하고, `fasten_test`는 체결 동작을 실행합니다. `--calib-vision`을 사용한 체결 보정에도 같은 `camera_topic`을 지정합니다.

`wrist_image_topic`을 쓰던 Launch와 GUI 단독 실행은 `camera_topic`으로 바꿉니다. 기존 진단 스크립트의 `--camera-topic` 및 Vision 1 보정의 `--topic`은 호환 입력입니다. ROS 파라미터와 함께 주면 `camera_topic` 파라미터가 우선합니다.

카메라 토픽이나 해상도를 바꾸면 검사 ROI와 체결 비전 보정이 실제 영상에 맞는지 확인합니다. GUI의 LIVE는 미리보기 수신 상태입니다. 체결용 영상 프레임의 신선도와 보정은 로봇 노드에서 검사합니다. PC를 나누어 실행할 때는 ROS Domain, RMW와 DDS 검색 범위를 맞춥니다.
