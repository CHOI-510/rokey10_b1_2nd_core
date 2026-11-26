FETCH_COMPLETE = '볼트와 공구를 인계했습니다. 작업이 끝나면 완료를 알려 주세요.'
PARSE_UNAVAILABLE = '명령을 확인할 수 없습니다. 다시 말하거나 화면에서 명령을 선택해 주세요.'
INSPECTION = {
    'top': '상단 검사 결과는 {}입니다.',
    'side': '측면 검사 결과는 {}입니다.',
}
HANDOFF_COMPLETE = '작업 완료를 확인했습니다. 이동 허가를 확인합니다.'
RESULT = {
    'PASS': '볼트 조립 검사를 통과했습니다.',
    'FAIL': '볼트 조립 검사에 실패했습니다.',
    'HOLD': '볼트 조립 작업을 보류했습니다.',
}


def result_text(outcome, reason):
    return f'{RESULT[outcome]} 사유 코드: {reason}'
