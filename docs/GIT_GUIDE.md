# Git 운영

| 브랜치 | 운영 |
|---|---|
| main | 검증 완료 버전 |
| develop | 관제 개발과 시스템 통합 |
| work | 팀원 공동 개발, 담당 폴더별 작업 |

원격 저장소 `CHOI-510/core_project`를 만든 뒤 사용자 확인 하에 프로젝트 루트에서 실행합니다. 첫 커밋은 main에 두고 같은 커밋에서 두 브랜치를 생성합니다.

```bash
git status --short
git branch -M main
git add -- .gitignore .env.example AGENTS.md README.md docs core_* tests data models
git diff --cached --stat
git commit -m "Build Core Project P0-a fake integration"
git branch develop main
git branch work main
git remote add origin git@github.com:CHOI-510/core_project.git
git push -u origin main
git push -u origin develop
git push -u origin work
```

팀원은 work에서 담당 폴더만 변경합니다. 팀 결과를 work → develop PR로 통합하고 공통 수정사항을 develop → work PR로 반영합니다. 통합 검증 후 develop → main PR을 병합합니다. 병합 충돌은 변경 담당자와 조율하고 일반 커밋으로 해결합니다. Force Push는 금지합니다.
