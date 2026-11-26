# 로드맵

| 단계 | 현재 상태와 남은 범위 |
|---|---|
| P0-a | 초기 Fake 조립 FSM, 공통 ROS 계약, JSONL·Episode 기록 구현. 현재 Fake와 실물은 공통 L/R FSM을 사용 |
| P0-b | 준비된 L/R 볼트에 대한 Vision 1 검사 → Doosan 체결 → 재검사 연결. L/R 외부 결과 수집 모드는 독립 체결 상태 공급자 필요. 현장에서는 로봇 서비스, 작업 좌표·툴·카메라 교정과 실물 정지·복구를 검증 |
| P0-c | 음성 알림 구독 및 LLM 서비스 계약 자리 구현. Wakeword, STT, 명령 해석·요약용 LLM과 TTS 연동 예정 |
| P1 | CORE MONITOR의 상태·결과·로그·명령·선택 영상 및 Wrench 패널 구현 및 Fake 검증. Episode 데이터 품질 검증, 선택 rosbag2 수집과 GUI 이력 조회 예정 |
| P2 이후 | DB·Web UI, Task Queue·Scheduler, 멀티카메라 Fusion, Tracking, 6D Pose, VLM, 분산 GPU 추론, 모방학습·강화학습 |

## 실물 통합 전 검토

- 어댑터의 독립 정지 요청 경로와 로봇 컨트롤러의 정지·유휴·힘·순응 상태를 검증합니다.
- 시계 동기화와 DDS 연결 단절, 서비스 지연, 노드 재시작을 분산 PC에서 검증하고 현장 측정에 따라 Timeout을 조정합니다.
- 기존 비전 PC 두 대로 FPS, 송출량, 손실 및 GUI Preview 사용 전후 영향을 측정합니다. [영상 파이프라인](VIDEO_PIPELINE.md)
- 음성·LLM 담당자는 명령 확인과 한국어 실패 안내를 연결합니다. [통신 계약](INTERFACES.md)
