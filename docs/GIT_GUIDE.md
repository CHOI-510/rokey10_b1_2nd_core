# Git 운영

원격 저장소는 `CHOI-510/rokey10_b1_2nd_core`입니다. 아래 명령은 팀원 공동 개발 브랜치를 체크아웃합니다. 빌드와 Fake 실행은 [README](../README.md)를 참고합니다.

```bash
git clone -b work https://github.com/CHOI-510/rokey10_b1_2nd_core.git
cd rokey10_b1_2nd_core
git status --short --branch
```

| 브랜치 | 운영 |
|---|---|
| main | 검증 완료 버전 |
| develop | 관제 개발과 시스템 통합 |
| work | 팀원 공동 개발, 담당 폴더별 작업 |

팀원은 work에서 담당 폴더만 변경합니다. 팀 결과를 work → develop PR로 통합하고 공통 수정사항을 develop → work PR로 반영합니다. 통합 검증 후 develop → main PR을 병합합니다. 병합 충돌은 변경 담당자와 조율하고 일반 커밋으로 해결합니다. Force Push는 금지합니다.
