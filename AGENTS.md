# Core Project 작업 지침

- [설계 동결 기준](docs/DESIGN_V0.1.md)과 [통신 계약](docs/INTERFACES.md)을 먼저 확인한다.
- ROS 패키지명은 core_*를 사용한다. 기존 외부 패키지 이름과 원본은 유지한다.
- Robot Node에 로봇 제어를 집중하고 모든 Action에서 공통 실행 잠금을 사용한다.
- P0-a 검증은 Fake 모드와 로컬 DDS로 수행한다. 기존 워크스페이스는 참조용으로 사용한다.
- 공통 인터페이스 변경은 근거와 호환성 영향을 DEVELOPMENT.md에 기록한다.
- 알고리즘 담당자는 [개발 가이드](docs/DEVELOPMENT.md)의 단독 검증 경로를 사용한다.
