# 로드맵

| 단계 | 현재 상태와 남은 범위 |
|---|---|
| P0-a | 단일 관제 FSM, Fake Robot·Vision, 통신 계약, JSONL·Episode 기록 구현. Fake 통합 시나리오 10개 검증 |
| P0-b | 두산 M0609·RG2 어댑터, 작업 좌표·툴·카메라 교정, 힘·토크 한계와 정지 회로 검증. D435i 등 실물 카메라와 힘 센서 발행기, YOLO 탐지·판정·영상 연동, 독립 그리퍼 상태 입력 구현 필요 |
| P0-c | 음성 알림 구독 및 LLM 서비스 계약 자리 구현. Wakeword, STT, 명령 해석·요약용 LLM과 TTS 연동 예정 |
| P1 | CORE MONITOR의 상태·결과·로그·명령·선택 영상 및 Wrench 패널 구현 및 Fake 검증. Episode 데이터 품질 검증, 선택 rosbag2 수집과 GUI 이력 조회 예정 |
| P2 이후 | DB·Web UI, Task Queue·Scheduler, 멀티카메라 Fusion, Tracking, 6D Pose, VLM, 분산 GPU 추론, 모방학습·강화학습 |

## 실물 통합 전 검토

- 관제의 로봇 정지·복귀 요청 Future에는 응답 기한이 없습니다. 응답이 유실되면 관제가 STOPPED에 머물며 후속 복구 요청이 진행되지 않을 수 있습니다. 무응답 처리 기한, 상태 재확인과 운영자 복구 절차를 정의하고 응답 유실·지연을 재현해 검증합니다.
- GUI는 관제 상태를 1초 동안 받지 못하면 FSM State를 NO SIGNAL로 바꾸고 STOP을 제외한 작업 버튼을 비활성화합니다. 이전 결과와 로그가 남을 수 있으므로 마지막 수신 시각과 오래된 데이터 표시, 연결 복구 후 상태 재동기화를 검토합니다.
- GUI의 관제 서비스 응답 기한은 5초입니다. 기한 초과 시 결과가 미확정이라고 기록합니다. 요청 결과가 뒤늦게 상태에 반영되는 경우의 화면 및 운영 절차를 검증합니다.

World Model, Sensor Manager, Network Manager, 별도 Recorder Node, Model Manager, API Gateway, 범용 Skill Registry와 Pydantic 기반 자동 Tool Schema는 향후 필요성이 확인되면 검토합니다.
