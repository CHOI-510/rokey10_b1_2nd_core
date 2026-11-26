# Vision 1 L/R 존재 검사

Vision 1은 준비된 작업물의 L/R 볼트 존재 여부를 `/integration/inspect_positions`로 반환합니다. 관제 요청의 `INSPECT`, `RECHECK`, `VERIFY`는 같은 서비스 타입을 사용합니다. `mode:=fake`에서는 Fake Vision이 같은 서비스를 제공하며, `mode:=real`에서는 통합 Launch가 실물 Vision 1을 기동합니다.

## 카메라와 ROI 준비

[Jazzy 빌드 절차](../README.md#설치-및-빌드)에 따라 CORE를 빌드하고 같은 오버레이를 source합니다. RealSense 원본 Image 발행기를 실행하고 실제 토픽을 확인합니다.

```bash
ros2 topic list -t
ros2 run core_vision vision1_calibrate --ros-args -p camera_topic:=/camera/camera/color/image_raw
```

검사 때 사용할 자세에서 실제 L(볼트 2), R(볼트 1) 순으로 클릭하고 S로 저장합니다. C는 선택을 초기화합니다. 기본 ROI는 `~/.config/core_vision/vision1.json`에 저장됩니다. 카메라·화각·해상도·토픽을 바꾸면 해당 구성으로 교정합니다. RGB 분류는 가림과 조명 변화에 민감하므로 실제 지그에서 좌우 누락과 판정 불가를 각각 검증합니다.

## 관제와 연결

- 실물 통합은 `ros2 launch core_bringup system.launch.py mode:=real gui:=true`에서 Vision 1을 기동합니다. 기준 토픽이 다르면 `camera_topic`을, 고정 검사 카메라를 쓸 때는 `vision_camera_topic`과 `vision_roi_file`을 지정합니다.
- 독립 Vision 1 시험과 `mode:=inspection` 연결에는 아래 명령을 사용합니다. 검사 전용 관제가 전체 결과를 내기 위해서는 별도 `/integration/read_fastening` 공급자도 필요합니다.

```bash
ros2 run core_vision vision1_node --ros-args -p real_camera:=true -p camera_topic:=/camera/camera/color/image_raw
```

실물 카메라를 연결한 경우에만 `real_camera:=true`를 사용합니다. 재생·합성 입력에 대한 독립 시험은 기본값 false를 사용합니다. 기본 프레임 수집 기한은 1초이며 관제 응답 기한 `inspectpositions_timeout`은 여유를 두고 설정합니다. 체결 토크·깊이는 이 서비스의 판정 대상에서 제외됩니다. [서비스 계약](../docs/INTERFACES.md#lr-최소-통합-계약)
