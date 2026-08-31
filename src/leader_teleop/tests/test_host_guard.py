"""
host의 일시적 오류 허용 로직 단위 테스트
========================================
TransientFailureGuard가 연속 실패를 한도까지 허용하고, 넘으면 원래
예외를 던지며, 성공 시 카운터를 되돌리는지 확인한다. 하드웨어 불필요.

실행:
	PYTHONPATH=src python -m leader_teleop.tests.test_host_guard
	(pytest 호환: PYTHONPATH=src pytest src/leader_teleop/tests)
"""

from leader_teleop.host import (
	MAX_CONSECUTIVE_COMMAND_FAILURES,
	MAX_CONSECUTIVE_OBSERVATION_FAILURES,
	TRANSIENT_ERRORS,
	TransientFailureGuard,
)


def test_tolerates_failures_up_to_limit() -> None:
	guard = TransientFailureGuard(limit=3, label='test')
	error = ConnectionError('no status packet')
	for _ in range(3):
		guard.record_failure(error)  # 3회까지는 예외 없음
	assert guard.count == 3


def test_raises_original_error_past_limit() -> None:
	guard = TransientFailureGuard(limit=2, label='test')
	error = ConnectionError('no status packet')
	guard.record_failure(error)
	guard.record_failure(error)
	try:
		guard.record_failure(error)
	except ConnectionError as raised:
		assert raised is error  # 감싸지 않고 원래 예외 그대로
	else:
		raise AssertionError('limit 초과 시 예외가 나야 한다')


def test_reset_clears_count() -> None:
	guard = TransientFailureGuard(limit=1, label='test')
	guard.record_failure(RuntimeError('glitch'))
	guard.reset()
	assert guard.count == 0
	guard.record_failure(RuntimeError('glitch'))  # 리셋 뒤 다시 1회 허용


def test_bus_connection_error_is_transient() -> None:
	# 실기에서 본 'There is no status packet'은 ConnectionError로 온다.
	assert ConnectionError in TRANSIENT_ERRORS
	assert TimeoutError in TRANSIENT_ERRORS  # 카메라 async_read 타임아웃
	assert MAX_CONSECUTIVE_OBSERVATION_FAILURES >= 5
	assert MAX_CONSECUTIVE_COMMAND_FAILURES >= 5


if __name__ == '__main__':
	for name, test in list(globals().items()):
		if name.startswith('test_') and callable(test):
			test()
			print(f'ok  {name}')
