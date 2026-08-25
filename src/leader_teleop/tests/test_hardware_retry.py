"""
connect_with_retry 순수 로직 단위 테스트
========================================
하드웨어 없이 재시도 정책을 검증한다. delay_s=0으로 sleep 없이 돈다.

실행:
	PYTHONPATH=src python -m leader_teleop.tests.test_hardware_retry
	(pytest 호환: PYTHONPATH=src pytest src/leader_teleop/tests)
"""

from leader_teleop.hardware_retry import connect_with_retry, is_voltage_error


class _FlakyOperation:
	"""처음 fail_count번 실패 후 성공하는 가짜 연결 시퀀스."""

	def __init__(self, fail_count: int, error: Exception) -> None:
		self.fail_count = fail_count
		self.error = error
		self.call_count = 0

	def __call__(self) -> None:
		self.call_count += 1
		if self.call_count <= self.fail_count:
			raise self.error


class _Counter:
	"""호출 횟수만 세는 콜백."""

	def __init__(self) -> None:
		self.call_count = 0

	def __call__(self) -> None:
		self.call_count += 1


def test_success_first_try_no_retry() -> None:
	"""첫 시도 성공이면 cleanup/diagnose 없이 1회만 호출된다."""
	operation = _FlakyOperation(0, ConnectionError('unused'))
	cleanup = _Counter()
	connect_with_retry(
		operation, label='test', cleanup=cleanup, delay_s=0.0
	)
	assert operation.call_count == 1
	assert cleanup.call_count == 0


def test_comm_glitch_recovers_after_retries() -> None:
	"""통신 글리치 2회 후 성공 - 재시도 전마다 cleanup이 불린다."""
	operation = _FlakyOperation(
		2, ConnectionError('Incorrect status packet!')
	)
	cleanup = _Counter()
	connect_with_retry(
		operation, label='test', cleanup=cleanup, delay_s=0.0
	)
	assert operation.call_count == 3
	assert cleanup.call_count == 2


def test_persistent_failure_raises_original_error() -> None:
	"""끝까지 실패하면 max_attempts만큼 시도 후 원래 예외를 던진다."""
	operation = _FlakyOperation(99, RuntimeError('Overload error!'))
	try:
		connect_with_retry(
			operation, label='test', max_attempts=3, delay_s=0.0
		)
	except RuntimeError as error:
		assert 'Overload' in str(error)
	else:
		raise AssertionError('RuntimeError가 전파돼야 한다')
	assert operation.call_count == 3


def test_voltage_error_triggers_diagnose() -> None:
	"""전압 에러면 실패한 시도마다 diagnose가 불린다."""
	operation = _FlakyOperation(1, RuntimeError('Input voltage error!'))
	diagnose = _Counter()
	connect_with_retry(
		operation, label='test', diagnose=diagnose, delay_s=0.0
	)
	assert operation.call_count == 2
	assert diagnose.call_count == 1


def test_non_hardware_error_propagates_immediately() -> None:
	"""ConnectionError/RuntimeError 외 예외는 재시도 없이 전파된다."""
	operation = _FlakyOperation(99, ValueError('bad config'))
	try:
		connect_with_retry(operation, label='test', delay_s=0.0)
	except ValueError:
		pass
	else:
		raise AssertionError('ValueError가 전파돼야 한다')
	assert operation.call_count == 1


def test_cleanup_failure_does_not_stop_retry() -> None:
	"""cleanup이 실패해도 재시도는 계속된다."""
	operation = _FlakyOperation(
		1, ConnectionError('Incorrect status packet!')
	)

	def broken_cleanup() -> None:
		raise OSError('port already closed')

	connect_with_retry(
		operation, label='test', cleanup=broken_cleanup, delay_s=0.0
	)
	assert operation.call_count == 2


def test_is_voltage_error_detection() -> None:
	"""전압 에러 판별은 메시지 문자열 기준으로 동작한다."""
	assert is_voltage_error(RuntimeError('... Input voltage error!'))
	assert not is_voltage_error(ConnectionError('Incorrect status packet!'))
	assert not is_voltage_error(RuntimeError('Overload error!'))


def main() -> None:
	"""전체 테스트를 실행하고 결과를 출력한다."""
	test_functions = [
		test_success_first_try_no_retry,
		test_comm_glitch_recovers_after_retries,
		test_persistent_failure_raises_original_error,
		test_voltage_error_triggers_diagnose,
		test_non_hardware_error_propagates_immediately,
		test_cleanup_failure_does_not_stop_retry,
		test_is_voltage_error_detection,
	]
	for test_function in test_functions:
		test_function()
		print(f'[PASS] {test_function.__name__}')
	print(f'\n{len(test_functions)}개 테스트 전부 통과')


if __name__ == '__main__':
	main()
