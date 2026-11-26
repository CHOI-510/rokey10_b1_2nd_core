# 로드맵

| 단계 | 범위 |
|---|---|
| P0-a | 관제 코어, Fake 검증, 통신 계약, JSONL, Episode 기록 규격과 담당자 템플릿 |
| P0-b | 두산/RG2 저수준 어댑터, D435i·RGB-D·YOLO·OpenCV, 교정과 승인된 안전 조건에서 실물 검증 |
| P0-c | Wakeword, STT, 명령 해석과 요약용 LLM, TTS |
| P1 | GUI 확장과 Episode 데이터 품질 검증, 선택 rosbag2 수집 |
| P2 이후 | DB·Web UI, Task Queue·Scheduler, 멀티카메라 Fusion, Tracking, 6D Pose, VLM, 분산 GPU 추론, 모방학습·강화학습 |

World Model, Sensor Manager, Network Manager, 별도 Recorder Node, Model Manager, API Gateway, 범용 Skill Registry와 Pydantic 기반 자동 Tool Schema는 향후 필요성이 확인되면 검토합니다.
