"""Match speech against the aliases stored in keywords-commands (no robot calls)."""

import re
import unicodedata
from collections.abc import Iterable, Mapping
from typing import Any


def aliases(value: Any) -> list[str]:
    """Accept comma-separated aliases, or an array of those strings."""
    values = value if isinstance(value, list) else [value]
    return list(dict.fromkeys(
        part.strip()
        for item in values if isinstance(item, str)
        for part in item.split(",") if part.strip()
    ))


def normalize(text: str) -> str:
    return unicodedata.normalize("NFKC", text).casefold()


def first_match(text: str, names: list[str]) -> int | None:
    positions = []
    for name in names:
        # Match standalone words and Korean particles/conjugations, while avoiding
        # e.g. driver inside screwdriver or 시작 inside 시작점.
        pattern = (
            r"(?<!\w)" + re.escape(normalize(name))
            + r"(?=$|[^\w]|(?:을|를|은|는|이|가|와|과|의|도|만|으로|로|에|에서|부터|까지|랑|들|해|하|했|합|할|되|됐)[가-힣]*(?!\w))"
        )
        match = re.search(pattern, text)
        if match:
            positions.append(match.start())
    return min(positions) if positions else None


def parse_transcription(text: str, documents: Iterable[Mapping[str, Any]]) -> dict:
    normalized = normalize(text)
    keywords = {}
    commands = {}
    for document in documents:
        names = aliases(document.get("keywords"))
        position = first_match(normalized, names)
        if position is not None:
            canonical = names[0]
            keywords.setdefault(normalize(canonical), (position, canonical))

        names = aliases(document.get("commands"))
        position = first_match(normalized, names)
        if position is not None:
            canonical = names[0]
            behave = document.get("behave")
            behave = behave.strip() if isinstance(behave, str) else ""
            # These messages simulate behavior only; database text is never executed.
            result = {
                "command": canonical,
                "behave": behave or None,
                "status": "simulated" if behave else "unconfigured",
                "message": (
                    f"{behave} 동작을 수행했습니다."
                    if behave else f"'{canonical}' 명령의 behave가 등록되지 않았습니다."
                ),
            }
            commands.setdefault((normalize(canonical), behave), (position, result))

    return {
        "keywords": [value for _, value in sorted(keywords.values(), key=lambda item: item[0])],
        "commands": [value for _, value in sorted(commands.values(), key=lambda item: item[0])],
    }
