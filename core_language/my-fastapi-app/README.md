# Python → FastAPI → Next.js

현재 예제는 `../test/stt_test.py`의 `STT.speech2text()`를 호출하고 반환한 문장을 웹에 표시합니다.

## 실행

`test/.env`에 `OPENAI_API_KEY=...`를 설정합니다. 키는 서버에서만 사용합니다.

터미널 1:

```bash
cd /home/lee/ws_cobot_pjt/ws_dsr/src/my-fastapi-app
uv sync
uv run uvicorn my_fastapi_app.main:app --reload --host 127.0.0.1 --port 8000
```

터미널 2:

```bash
cd /home/lee/ws_cobot_pjt/ws_dsr/src/my-next-app
npm run dev
```

http://localhost:3000 에서 **5초 녹음 시작**을 누릅니다.
FastAPI가 실행 중인 PC의 마이크를 5초 녹음한 뒤 OpenAI로 음성을 전송해 텍스트로 변환합니다.
브라우저가 다른 기기에서 실행되어도 녹음하는 장치는 서버 PC입니다.

## 연결 구조

1. `my-next-app/app/page.tsx`가 `POST /api/stt`를 호출합니다.
2. `my-next-app/next.config.ts`가 요청을 `http://127.0.0.1:8000/api/stt`로 전달합니다.
3. `src/my_fastapi_app/main.py`가 `test/stt_test.py`의 함수를 실행합니다.
4. FastAPI의 `{"text": "변환 결과"}` 응답을 웹이 표시합니다.

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
