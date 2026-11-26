# Git 운영

CORE 팀의 공통 기준은 `main`입니다. 빌드와 실행은 [README](../README.md)를 참고합니다.

```bash
git clone https://github.com/CHOI-510/rokey10_b1_2nd_core.git
cd rokey10_b1_2nd_core
git status --short --branch
```

담당 작업은 최신 `main`에서 작업 브랜치를 만들어 시작합니다. 통합 변경을 공유할 때는 대상 브랜치를 `main`으로 지정하고, 변경된 ROS 패키지와 테스트 경로를 함께 검토합니다.

기존 `develop`, `work`, `release/core-integration`에는 초기 팀 작업과 실험 이력이 있습니다. 필요한 변경은 파일의 담당 패키지를 확인한 뒤 가져옵니다. 음성 Web 실험 소스는 기존 팀 브랜치에서 볼 수 있습니다. 코어의 음성 계약은 `core_language`를 사용합니다.
