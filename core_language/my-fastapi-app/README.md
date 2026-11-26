# Python → FastAPI → Next.js

`../test/stt_test.py`의 `STT.speech2text()`로 변환한 문장을 MongoDB에 저장하고,
같은 DB의 `keywords-commands` 컬렉션에서 키워드·명령을 추출해 웹에 표시합니다.

## 실행

`test/.env`에 `OPENAI_API_KEY=...`를 설정합니다. 키는 서버에서만 사용합니다.

터미널 1:

```bash
cd /home/lee/ws_cobot_pjt/ws_dsr/src/rokey10_b1_2nd_core/core_language/my-fastapi-app
uv sync
uv run uvicorn my_fastapi_app.main:app --reload --host 127.0.0.1 --port 8000
```

터미널 2:

```bash
cd /home/lee/ws_cobot_pjt/ws_dsr/src/rokey10_b1_2nd_core/core_language/my-next-app
npm run dev
```

http://localhost:3000 에서 **5초 녹음 시작**을 누릅니다.
FastAPI가 실행 중인 PC의 마이크를 5초 녹음한 뒤 OpenAI로 음성을 전송해 텍스트로 변환합니다.
브라우저가 다른 기기에서 실행되어도 녹음하는 장치는 서버 PC입니다.

## 연결 구조

1. `my-next-app/app/page.tsx`가 `POST /api/stt`를 호출합니다.
2. `my-next-app/next.config.ts`가 요청을 `http://127.0.0.1:8000/api/stt`로 전달합니다.
3. `src/my_fastapi_app/main.py`가 `test/stt_test.py`의 함수를 실행합니다.
4. 기존 `MONGODB_COLLECTION`에 원문을 저장합니다.
5. `keywords-commands`에서 등록 정보를 조회해 파싱합니다.
6. 웹이 원문, 추출한 키워드, 명령과 `behave`의 모의 수행 메시지를 표시합니다.

## 키워드·명령 등록 형식

`my-fastapi-app/.env`의 기존 `MONGODB_URI`, `MONGODB_DATABASE`,
`MONGODB_COLLECTION` 설정을 사용합니다. 규칙 컬렉션은 기본적으로
`keywords-commands`이며 `MONGODB_RULES_COLLECTION`로 변경할 수 있습니다.
음성 원문 저장 컬렉션과 규칙 컬렉션은 별도로 지정하세요.

```json
{"keywords": "Hammer, 해머, 망치"}
{"keywords": "screwdriver, driver, 드라이버"}
{"commands": "시작", "behave": "검사 시작"}
{"commands": "완료", "behave": "작업 상태 체크"}
{"commands": "취소", "behave": "작업 취소"}
```

쉼표로 구분한 이름 또는 문자열 배열을 동의어로 사용하며, 첫 이름을 대표 이름으로 반환합니다.
영문 대소문자와 한국어 조사·일부 활용형을 처리합니다. 예를 들어
`망치로 검사 시작해줘`는 `Hammer` 키워드와 `시작` 명령을 추출합니다.
같은 항목은 한 번만 표시하고 여러 항목은 발화 순서로 반환합니다.
매 요청마다 DB를 조회하므로 등록 내용을 바꾸면 다음 녹음부터 적용됩니다.
이는 등록된 표현의 문자열 매칭으로, 부정 표현이나 복잡한 문맥의 의도를 판별하지는 않습니다.

응답의 `text`는 유지하며 `keywords`, `commands`, `extraction_error`를 추가합니다.

```json
{
  "text": "망치로 검사 시작해줘",
  "keywords": ["Hammer"],
  "commands": [{
    "command": "시작",
    "behave": "검사 시작",
    "status": "simulated",
    "message": "검사 시작 동작을 수행했습니다."
  }],
  "extraction_error": null
}
```

동작 수행은 **UI 시뮬레이션**입니다. 실제 로봇/ROS 명령을 실행하지 않습니다.
`behave`가 없으면 `status=unconfigured`와 설정 누락 메시지를 반환합니다.
매칭 항목이 없으면 빈 배열을 반환합니다. 원문 저장 후 규칙 조회가 실패하면
원문과 `extraction_error`를 반환해 UI에 저장된 원문과 오류를 함께 표시합니다.

파서 및 API 회귀 검사(실제 녹음·DB 쓰기·OpenAI 호출 없음):

```bash
uv run python -m unittest discover -s tests -v
```

Python 처리 내용을 바꾸려면 `test/stt_test.py`의 `speech2text()`를 수정하세요.
다른 형태의 결과를 반환하려면 FastAPI의 응답 모델과 Next.js의 표시 코드도 함께 수정합니다.
두 프로젝트와 `test` 폴더는 같은 상위 폴더에 두어야 합니다.

다른 백엔드 주소를 쓰려면 Next.js의 `.env.local`에 `FASTAPI_URL=http://주소:포트`를 설정한 뒤 Next.js를 재시작하세요.
마이크를 지정하려면 서버 환경 변수 또는 `test/.env`에 `VOICE_MIC_DEVICE`를 설정합니다.
마이크 사용에는 운영체제의 PortAudio 라이브러리가 필요합니다.
현재 마이크 중복 사용 방지는 단일 서버 프로세스 기준이므로 위 실행 명령을 사용하세요.

## 확인한 항목

- Next.js ESLint 및 TypeScript 검사
- 실제 STT 대신 모의 결과를 사용한 FastAPI 성공 응답
- API 키 미설정(503), 중복 녹음(409), 변환 오류(500), 오류 후 잠금 해제

실제 녹음 및 OpenAI 음성 변환 호출은 별도로 확인해야 합니다.
