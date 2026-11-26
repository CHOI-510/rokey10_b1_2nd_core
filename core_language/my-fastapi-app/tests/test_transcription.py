import importlib
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient
from pymongo.errors import PyMongoError

from my_fastapi_app.command_parser import parse_transcription


RULES = [
    {"keywords": "Hammer, 해머, 망치"},
    {"keywords": "screwdriver, driver, 드라이버"},
    {"keywords": "bolt, 볼트"},
    {"keywords": "nut, 너트"},
    {"commands": "시작", "behave": "검사 시작"},
    {"commands": "완료", "behave": "작업 상태 체크"},
    {"commands": "취소", "behave": "작업 취소"},
]


class ParserTests(unittest.TestCase):
    def test_real_schema_and_korean_suffixes(self):
        result = parse_transcription("망치와 드라이버로 볼트를 검사 시작해줘", RULES)
        self.assertEqual(result["keywords"], ["Hammer", "screwdriver", "bolt"])
        self.assertEqual(result["commands"][0]["behave"], "검사 시작")
        self.assertEqual(result["commands"][0]["status"], "simulated")

    def test_case_alias_deduplication_and_speech_order(self):
        result = parse_transcription("너트와 HAMMER, 망치, 해머. 완료하고 시작해주세요", RULES)
        self.assertEqual(result["keywords"], ["nut", "Hammer"])
        self.assertEqual([c["command"] for c in result["commands"]], ["완료", "시작"])

    def test_unrelated_words_do_not_match(self):
        self.assertEqual(parse_transcription("butternut bolting 시작점 취소선", RULES),
                         {"keywords": [], "commands": []})
        self.assertEqual(parse_transcription("screwdriver", [{"keywords": "driver"}])["keywords"], [])

    def test_array_aliases_and_missing_behavior(self):
        result = parse_transcription("멈춰", [{"commands": ["정지", "멈춰"]}])
        self.assertEqual(result["commands"][0]["command"], "정지")
        self.assertEqual(result["commands"][0]["status"], "unconfigured")
        self.assertNotIn("수행했습니다", result["commands"][0]["message"])

    def test_empty_or_invalid_rules(self):
        self.assertEqual(parse_transcription("안녕하세요", [{"keywords": None}, {"commands": 123}]),
                         {"keywords": [], "commands": []})


class TranscriptionApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Import without any real MongoDB network access, microphone or API calls.
        with patch("pymongo.MongoClient"):
            main = importlib.import_module("my_fastapi_app.main")
        cls.main = main
        cls.client = TestClient(main.app)

    def setUp(self):
        self.saved = MagicMock()
        self.rules = MagicMock()
        self.rules.find.return_value = RULES
        self.stt = MagicMock()
        self.stt.return_value.speech2text.return_value = "망치로 검사 시작해줘"
        self.patches = [
            patch.object(self.main, "transcriptions", self.saved),
            patch.object(self.main, "command_rules", self.rules),
            patch.dict(sys.modules, {"stt_test": SimpleNamespace(STT=self.stt, openai_api_key="fake")}),
        ]
        for active_patch in self.patches:
            active_patch.start()
            self.addCleanup(active_patch.stop)

    def test_saves_transcript_and_returns_extraction(self):
        response = self.client.post("/api/stt")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["keywords"], ["Hammer"])
        self.assertEqual(response.json()["commands"][0]["behave"], "검사 시작")
        self.saved.insert_one.assert_called_once()
        self.assertEqual(self.saved.insert_one.call_args.args[0]["text"], "망치로 검사 시작해줘")

    def test_real_stt_import_and_endpoint(self):
        # Exercise the real module imports; mock only recording/API work.
        self.patches[2].stop()
        stt_module = importlib.import_module("stt_test")
        self.assertTrue(callable(stt_module.resolve_input_device))
        with patch.object(stt_module, "STT", self.stt), patch.object(stt_module, "openai_api_key", "fake"):
            response = self.client.post("/api/stt")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["keywords"], ["Hammer"])

    def test_rule_lookup_failure_preserves_saved_text(self):
        self.rules.find.side_effect = PyMongoError("unavailable")
        response = self.client.post("/api/stt")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["text"], "망치로 검사 시작해줘")
        self.assertTrue(response.json()["extraction_error"])
        self.saved.insert_one.assert_called_once()
        self.assertFalse(self.main.recording_lock.locked())

    def test_storage_failure_returns_error_and_unlocks(self):
        self.saved.insert_one.side_effect = PyMongoError("unavailable")
        self.assertEqual(self.client.post("/api/stt").status_code, 503)
        self.assertFalse(self.main.recording_lock.locked())

    def test_busy_and_empty_speech_do_not_save(self):
        self.main.recording_lock.acquire()
        try:
            self.assertEqual(self.client.post("/api/stt").status_code, 409)
        finally:
            self.main.recording_lock.release()
        self.stt.return_value.speech2text.return_value = " "
        self.assertEqual(self.client.post("/api/stt").status_code, 422)
        self.saved.insert_one.assert_not_called()


if __name__ == "__main__":
    unittest.main()
