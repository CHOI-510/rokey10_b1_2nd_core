import logging
import sys
from pathlib import Path
from threading import Lock

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
from typing import Literal

from my_fastapi_app.command_parser import parse_transcription

import os
from datetime import datetime, timezone

from dotenv import load_dotenv
from pymongo import MongoClient
from pymongo.errors import PyMongoError

# Python processing lives in the sibling test directory.
TEST_DIR = Path(__file__).resolve().parents[3] / "test"
sys.path.insert(0, str(TEST_DIR))

# my-fastapi-app/.env를 읽습니다.
load_dotenv(Path(__file__).resolve().parents[2] / ".env")

mongo_client = MongoClient(
    os.getenv("MONGODB_URI", "mongodb://localhost:27017"),
    serverSelectionTimeoutMS=5000,
    tz_aware=True,
)

database = mongo_client[os.getenv("MONGODB_DATABASE", "voice_app")]
transcriptions = database[
    os.getenv("MONGODB_COLLECTION", "transcriptions")
]
command_rules = database[
    os.getenv("MONGODB_RULES_COLLECTION", "keywords-commands")
]

app = FastAPI()
recording_lock = Lock()
logger = logging.getLogger(__name__)


class CommandResult(BaseModel):
    command: str
    behave: str | None
    status: Literal["simulated", "unconfigured"]
    message: str


class TranscriptionResponse(BaseModel):
    text: str
    keywords: list[str] = Field(default_factory=list)
    commands: list[CommandResult] = Field(default_factory=list)
    extraction_error: str | None = None


@app.get("/")
def root():
    return {"message": "Hello FastAPI"}


@app.post("/api/stt", response_model=TranscriptionResponse)
def transcribe():
    if not recording_lock.acquire(blocking=False):
        raise HTTPException(status_code=409, detail="이미 녹음 중입니다. 잠시 후 다시 시도하세요.")
    try:
        from stt_test import STT, openai_api_key

        if not openai_api_key:
            raise HTTPException(status_code=503, detail="test/.env에 OPENAI_API_KEY를 설정하세요.")

        # return TranscriptionResponse(text=STT(openai_api_key).speech2text()) api키를 활용하여 stt하는 작업
        text = STT(openai_api_key).speech2text()

        if not text.strip():
            raise HTTPException(
                status_code=422,
                detail="인식된 음성이 없습니다. 다시 녹음해 주세요.",
            )

        try:
            transcriptions.insert_one({
                "text": text,
                "created_at": datetime.now(timezone.utc),
            })
        except PyMongoError:
            logger.exception("MongoDB 저장 실패")
            raise HTTPException(
                status_code=503,
                detail="음성은 변환됐지만 DB 저장에 실패했습니다. MongoDB 연결을 확인하세요.",
            ) from None

        try:
            # Read fresh rules each time, so DB edits apply to the next recording.
            extracted = parse_transcription(text, command_rules.find(
                {}, {"_id": 0, "keywords": 1, "commands": 1, "behave": 1}
            ))
        except PyMongoError:
            logger.exception("MongoDB 키워드/명령 조회 실패")
            return TranscriptionResponse(
                text=text,
                extraction_error="음성은 저장됐지만 키워드·명령 조회에 실패했습니다. 다시 녹음해 주세요.",
            )

        return TranscriptionResponse(text=text, **extracted)
    except HTTPException:
        raise
    except Exception:
        logger.exception("Speech transcription failed")
        raise HTTPException(
            status_code=500,
            detail="음성 변환에 실패했습니다. 서버의 마이크, API 키 및 실행 로그를 확인하세요.",
        ) from None
    finally:
        recording_lock.release()
