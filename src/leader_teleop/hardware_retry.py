"""
하드웨어 연결 재시도 유틸리티
=============================
Feetech 버스는 연결/설정 단계에서 간헐적으로 두 유형의 실패를 일으킨다:

- **통신 글리치** (`ConnectionError`, 'Incorrect status packet') — 응답
  패킷이 깨진 경우. 재시도로 거의 항상 회복된다.
- **모터 에러 비트** (`RuntimeError`, 'Input voltage error' 등) — 모터가
  상태 패킷에 하드웨어 이상을 보고한 경우. 일시적 전압 딥이면 재시도로
  회복되지만, 반복되면 전원/배선 문제다 (SO-101 리더암은 5V, 팔로워는
  12V — lerobot 이슈 #2387의 확정 원인).

lerobot의 write 계층 `num_retry`는 통신 글리치만 재시도하고 모터 에러
비트는 즉시 예외를 던지므로(motors_bus._write), 연결 시퀀스 전체를
감싸는 상위 재시도가 필요하다. 최대 5회는 lerobot 커뮤니티(이슈 #3131)
와 upstream(disconnect num_retry=5) 관례값이다.
"""

import time
from collections.abc import Callable
from typing import Final

# 연결 시퀀스 최대 시도 횟수 (lerobot 커뮤니티/upstream 관례값).
MAX_CONNECT_ATTEMPTS: Final[int] = 5
# 재시도 사이 대기 시간 (s) - 일시적 전압 딥/버스 노이즈가 가라앉을 여유.
CONNECT_RETRY_DELAY_S: Final[float] = 1.0
# 개별 버스 write/read에 줄 재시도 횟수 (통신 글리치 흡수용).
BUS_WRITE_NUM_RETRY: Final[int] = 2

# 전압 에러 시 출력할 점검 안내 (SO-101: 리더 5V / 팔로워 12V).
VOLTAGE_HINT: Final[str] = (
	'[warn] A motor reported an input voltage error. Check the power '
	'supply: SO-101 leader arms use 5V and follower arms use 12V. '
	'Also reseat the daisy-chain connectors (the last motor in the '
	'chain sags the most).'
)


def is_voltage_error(error: BaseException) -> bool:
	"""예외가 모터의 입력 전압 에러 보고인지 판별한다.

	Args:
		error: 버스 write/read가 던진 예외.

	Returns:
		메시지에 전압 에러가 언급되면 True.
	"""
	return 'voltage' in str(error).lower()


def log_bus_voltages(bus, label: str) -> None:
	"""버스의 모든 모터 입력 전압을 읽어 출력한다 (진단용, best-effort).

	전압 에러가 전원 문제인지 판정할 실측 근거를 남긴다. 읽기 실패는
	모터별로 무시한다.

	Args:
		bus: lerobot FeetechMotorsBus (포트가 열린 상태).
		label: 로그에 붙일 버스 이름 (예: 'bus1', 'left leader').
	"""
	for motor in bus.motors:
		try:
			raw_voltage = bus.read(
				'Present_Voltage', motor, normalize=False,
				num_retry=BUS_WRITE_NUM_RETRY,
			)
			print(f'[diag] {label} {motor}: {raw_voltage / 10:.1f}V')
		except Exception:
			print(f'[diag] {label} {motor}: voltage read failed')


def connect_with_retry(
	operation: Callable[[], None],
	label: str,
	cleanup: Callable[[], None] | None = None,
	diagnose: Callable[[], None] | None = None,
	max_attempts: int = MAX_CONNECT_ATTEMPTS,
	delay_s: float = CONNECT_RETRY_DELAY_S,
) -> None:
	"""간헐 실패를 흡수하도록 연결/설정 시퀀스를 재시도한다.

	통신 글리치(ConnectionError)와 모터 에러 비트(RuntimeError)만
	재시도 대상이며, 그 외 예외는 그대로 전파된다. 전압 에러가 감지되면
	매번 전원 점검 안내를 출력하고 diagnose를 호출해 실측 전압을 남긴다.

	Args:
		operation: 연결/설정 시퀀스 (재호출해도 안전해야 함).
		label: 로그에 붙일 작업 이름.
		cleanup: 재시도 전에 실행할 정리 (열린 포트 닫기 등, best-effort).
		diagnose: 전압 에러일 때 실행할 진단 (전압 로그 등, best-effort).
		max_attempts: 최대 시도 횟수.
		delay_s: 재시도 사이 대기 시간 (s).

	Raises:
		ConnectionError: 마지막 시도까지 통신이 실패한 경우.
		RuntimeError: 마지막 시도까지 모터가 에러를 보고한 경우.
	"""
	for attempt in range(1, max_attempts + 1):
		try:
			operation()
			return
		except (ConnectionError, RuntimeError) as error:
			if is_voltage_error(error):
				print(VOLTAGE_HINT)
				if diagnose is not None:
					try:
						diagnose()
					except Exception:
						print(f'[diag] {label}: diagnosis failed')
			if attempt == max_attempts:
				raise
			print(
				f'[warn] {label} failed '
				f'(attempt {attempt}/{max_attempts}): {error}'
			)
			if cleanup is not None:
				try:
					cleanup()
				except Exception:
					pass
			time.sleep(delay_s)
