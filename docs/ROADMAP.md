# 로드맵

| 단계 | 현재 상태와 남은 범위 |
|---|---|
| P0-a | 단일 관제 FSM, Fake Robot·Vision, 통신 계약, JSONL·Episode 기록 구현. Fake 통합 시나리오 10개 검증 |
| P0-b | 두산 M0609·RG2 어댑터, 작업 좌표·툴·카메라 교정, 힘·토크 한계와 정지 회로 검증. D435i 등 실물 카메라와 힘 센서 발행기, YOLO 탐지·판정·영상 연동, 독립 그리퍼 상태 입력 구현 필요 |
| P0-c | 음성 알림 구독 및 LLM 서비스 계약 자리 구현. Wakeword, STT, 명령 해석·요약용 LLM과 TTS 연동 예정 |
| P1 | CORE MONITOR의 상태·결과·로그·명령·선택 영상 및 Wrench 패널 구현 및 Fake 검증. Episode 데이터 품질 검증, 선택 rosbag2 수집과 GUI 이력 조회 예정 |
| P2 이후 | DB·Web UI, Task Queue·Scheduler, 멀티카메라 Fusion, Tracking, 6D Pose, VLM, 분산 GPU 추론, 모방학습·강화학습 |

## 실물 통합 전 검토

- 어댑터의 독립 정지 요청 경로와 로봇 컨트롤러의 정지·유휴·힘·순응 상태를 검증합니다.
- 시계 동기화와 DDS 연결 단절, 서비스 지연, 노드 재시작을 분산 PC에서 검증하고 현장 측정에 따라 Timeout을 조정합니다.
- 기존 비전 PC 두 대로 FPS, 송출량, 손실 및 GUI Preview 사용 전후 영향을 측정합니다. [영상 파이프라인](VIDEO_PIPELINE.md)
- 음성·LLM 담당자는 명령 확인과 한국어 실패 안내를 연결합니다. [통신 계약](INTERFACES.md)
