"""
원격 host 진입점 (라즈베리파이에서 실행)
========================================
파이에 연결된 실물 양팔 팔로워(+카메라)를 열고, ZMQ로 관측을 내보내며
PC의 클라이언트(`--robot.type=bi_so101_client`)가 보내는 관절 명령을
받아 구동합니다. 리더암·키보드·데이터셋 저장은 전부 PC 쪽 몫이고,
host는 "로봇 드라이버를 네트워크로 연장"하는 역할만 합니다.

	PC (teleoperate/record + 리더암)  <--ZMQ-->  Pi (host + 팔로워·카메라)

CLI는 다른 진입점과 같은 draccus 체계입니다 (`--robot.*` /
`--camera_head.mode` / `--home_return.*` / `--host.*`). 머리 모터 장착
여부는 `--camera_head.mode`(none이면 미장착)로 정하며, PC 쪽에서도 같은
장착 여부를 줘야 관측/액션 키가 일치합니다.

실행 예 (파이에서, 레포 루트):

	PYTHONPATH=src python -m leader_teleop.host \\
		--robot.type=bi_so101_follower --robot.id=bi_so101_follower \\
		--robot.left_arm_port=/dev/ttyACM0 \\
		--robot.right_arm_port=/dev/ttyACM1 \\
		--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0,
			"width": 640, "height": 480, "fps": 30}}'

종료는 Ctrl+C(SIGTERM·SSH 끊김도 같게 처리). 홈포즈 복귀 후 토크가
풀립니다. PC 쪽 프로그램이 먼저 끝나도 host는 계속 돌며 팔을 마지막
자세로 유지합니다.
"""

import json
import logging
import math
import signal
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat
from types import FrameType
from typing import Any, Final

import zmq

from lerobot.configs import parser
from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging

from .app import TeleopStackConfig
from .remote_codec import (
	DEFAULT_JPEG_QUALITY,
	DEFAULT_PORT_ZMQ_CMD,
	DEFAULT_PORT_ZMQ_OBSERVATIONS,
	encode_observation,
)
from .robots import BiFollowerBase, BiFollowerBaseConfig, BiSo101FollowerConfig


logger = logging.getLogger(__name__)

# 관측 읽기 실패(카메라 타임아웃, 버스 응답 없음)를 직전 관측 재사용으로
# 버티는 연속 횟수. 카메라 타임아웃(200ms)이면 약 2초, 버스 글리치(수십
# ms)면 1초 미만에 해당한다. 넘으면 장애로 보고 host를 종료한다.
# 재사용 구간의 관측은 중복 프레임이다.
MAX_CONSECUTIVE_OBSERVATION_FAILURES: Final[int] = 10
# 명령 전송(버스 쓰기) 실패를 건너뛰며 버티는 연속 횟수. 30Hz 루프 기준
# 약 0.5초. 첫 명령 순간의 전류 피크로 버스가 잠깐 침묵하는 경우를 넘긴다.
MAX_CONSECUTIVE_COMMAND_FAILURES: Final[int] = 15
# 일시적 오류로 취급해 재시도할 예외. 그 밖의 예외는 즉시 종료한다.
TRANSIENT_ERRORS: Final[tuple[type[BaseException], ...]] = (
	TimeoutError, RuntimeError, ConnectionError,
)
# 루프 주기 통계를 집계·보고하는 간격 (s).
LOOP_RATE_REPORT_INTERVAL_S: Final[float] = 5.0
# 실측 주기가 목표의 이 비율 아래면 경고한다.
LOOP_RATE_WARN_RATIO: Final[float] = 0.8


@dataclass
class HostConfig:
	"""host 통신 설정 (--host.*)."""

	port_zmq_cmd: int = DEFAULT_PORT_ZMQ_CMD  # 명령 수신 포트
	port_zmq_observations: int = DEFAULT_PORT_ZMQ_OBSERVATIONS  # 관측 송신 포트
	fps: int = 30  # 명령 처리·관측 송신 주기 (Hz)
	jpeg_quality: int = DEFAULT_JPEG_QUALITY  # 카메라 프레임 JPEG 품질


@dataclass
class HostAppConfig(TeleopStackConfig):
	"""host 실행 설정 (camera_head/home_return은 TeleopStackConfig)."""

	# 실물 로봇 (--robot.type=bi_so101_follower 등)
	robot: RobotConfig = field(default_factory=BiSo101FollowerConfig)
	# 통신 (--host.*)
	host: HostConfig = field(default_factory=HostConfig)


def _raise_keyboard_interrupt(signum: int, frame: FrameType | None) -> None:
	"""SIGTERM/SIGHUP을 Ctrl+C와 같은 정리 경로로 보낸다."""
	raise KeyboardInterrupt(f'signal {signum}')


class TransientFailureGuard:
	"""연속 실패 횟수를 세어 일시적 오류와 지속 장애를 구분한다.

	Attributes:
		limit: 허용하는 최대 연속 실패 횟수. 넘으면 예외를 다시 던진다.
		label: 로그에 표시할 대상 이름.
		count: 현재 연속 실패 횟수.
	"""

	def __init__(self, limit: int, label: str) -> None:
		self.limit = limit
		self.label = label
		self.count = 0

	def record_failure(self, error: BaseException) -> None:
		"""실패 1회를 기록한다. 한도를 넘으면 원래 예외를 다시 던진다.

		첫 실패에만 경고를 남겨 로그가 넘치지 않게 한다.

		Raises:
			BaseException: 연속 실패가 limit를 넘었을 때 넘겨받은 error.
		"""
		self.count += 1
		if self.count > self.limit:
			raise error
		if self.count == 1:
			logger.warning(
				f'{self.label} failed ({error}) - tolerating up to '
				f'{self.limit} consecutive failures.'
			)

	def reset(self) -> None:
		"""성공 시 호출. 실패 뒤 복구였으면 알린다."""
		if self.count:
			logger.info(
				f'{self.label} recovered after {self.count} failure(s).'
			)
		self.count = 0


def _is_valid_command(action: Any) -> bool:
	"""네트워크로 들어온 명령이 '문자열 -> 유한한 수' 딕셔너리인지 검사."""
	return isinstance(action, dict) and all(
		isinstance(key, str)
		and isinstance(value, (int, float))
		and not isinstance(value, bool)  # bool은 int의 서브클래스
		and math.isfinite(value)
		for key, value in action.items()
	)


class RobotHost:
	"""실물 로봇을 ZMQ로 노출하는 서버 루프.

	Attributes:
		config: 실행 설정.
		robot: 이 파이에 연결된 실물 로봇 (BiFollowerBase 계열).
	"""

	def __init__(self, config: HostAppConfig) -> None:
		self.config = config
		if not isinstance(config.robot, BiFollowerBaseConfig):
			raise ValueError(
				'host needs a physical follower robot '
				'(--robot.type=bi_so101_follower / bi_so102_follower), '
				f"got '{config.robot.type}'."
			)
		# 머리 장착 여부는 app과 같은 규칙(--camera_head.mode)으로 정한다.
		config.robot.has_head_motors = (
			config.camera_head.mode.lower() != 'none'
		)
		self.robot = make_robot_from_config(config.robot)
		self._zmq_context = zmq.Context()
		self._cmd_socket = self._zmq_context.socket(zmq.PULL)
		self._cmd_socket.setsockopt(zmq.CONFLATE, 1)
		self._observation_socket = self._zmq_context.socket(zmq.PUSH)
		self._observation_socket.setsockopt(zmq.CONFLATE, 1)
		self._is_shutdown_done = False
		self._has_warned_bad_command = False
		self._observation_guard = TransientFailureGuard(
			MAX_CONSECUTIVE_OBSERVATION_FAILURES, 'Observation read'
		)
		self._command_guard = TransientFailureGuard(
			MAX_CONSECUTIVE_COMMAND_FAILURES, 'Command send'
		)
		# 루프 주기 통계 (LOOP_RATE_REPORT_INTERVAL_S마다 집계).
		self._rate_window_start = 0.0
		self._rate_window_iterations = 0

	def setup(self) -> 'RobotHost':
		"""로봇을 연결하고 포트를 연다 (체이닝용)."""
		try:
			logger.info('Connecting robot...')
			self.robot.connect()
			self._cmd_socket.bind(f'tcp://*:{self.config.host.port_zmq_cmd}')
			self._observation_socket.bind(
				f'tcp://*:{self.config.host.port_zmq_observations}'
			)
			return self
		except BaseException:
			logger.warning('Init interrupted/failed - disabling robot torque.')
			self.shutdown()
			raise

	def run(self) -> None:
		"""명령 수신 → 구동 → 관측 송신 루프 (Ctrl+C/SIGTERM으로 종료)."""
		camera_keys = tuple(self.robot.cameras)
		dt_nominal = 1.0 / self.config.host.fps
		last_observation: dict[str, Any] | None = None
		sequence = 0
		is_client_receiving: bool | None = None
		self._rate_window_start = time.perf_counter()
		try:
			logger.info(
				f'Serving on ports {self.config.host.port_zmq_cmd}(cmd) / '
				f'{self.config.host.port_zmq_observations}(obs). '
				'Waiting for client... (Ctrl+C to stop)'
			)
			while True:
				loop_start = time.perf_counter()

				self._apply_latest_command()

				# 카메라 타임아웃이나 버스 응답 없음 한 번으로 host 전체가
				# 죽지 않게 직전 관측을 재사용한다. 연속 실패가 한도를
				# 넘으면 장애로 보고 종료한다.
				try:
					last_observation = self.robot.get_observation()
					self._observation_guard.reset()
				except TRANSIENT_ERRORS as error:
					if last_observation is None:
						raise
					self._observation_guard.record_failure(error)

				sequence += 1
				message = encode_observation(
					last_observation, camera_keys,
					self.config.host.jpeg_quality,
					timestamp=time.time(), sequence=sequence,
				)
				try:
					self._observation_socket.send_string(message, zmq.NOBLOCK)
					is_sent = True
				except zmq.Again:
					is_sent = False
				if is_sent != is_client_receiving:
					is_client_receiving = is_sent
					logger.info(
						'Client connected - streaming observations.'
						if is_sent else 'No client receiving observations.'
					)

				self._track_loop_rate(loop_start)
				elapsed = time.perf_counter() - loop_start
				precise_sleep(max(0.0, dt_nominal - elapsed))
		except KeyboardInterrupt:
			logger.info('Shutting down safely.')
		finally:
			self.shutdown()

	def _apply_latest_command(self) -> None:
		"""도착한 최신 명령을 검증해 로봇에 보낸다 (없으면 자세 유지).

		형식이 틀린 명령은 버리고(1회 경고), 모터 통신 예외는 연속 한도
		안에서는 건너뛰고 한도를 넘으면 전파해 host가 정리 경로(홈포즈
		복귀·토크 해제)로 빠지게 한다.
		"""
		try:
			message = self._cmd_socket.recv_string(zmq.NOBLOCK)
		except zmq.Again:
			return  # 새 명령 없음 - 팔은 마지막 자세를 유지한다
		try:
			action = json.loads(message)
			if not _is_valid_command(action):
				raise ValueError('command must map str -> finite number')
		except (ValueError, TypeError) as error:
			if not self._has_warned_bad_command:
				self._has_warned_bad_command = True
				logger.error(f'Bad command dropped: {error}')
			return
		self._has_warned_bad_command = False
		if not action:
			return
		# 버스 쓰기 실패(전류 피크로 인한 순간 침묵 등)는 이번 명령을
		# 건너뛰고 다음 명령으로 넘어간다. 지속되면 종료한다.
		try:
			self.robot.send_action(action)
			self._command_guard.reset()
		except ConnectionError as error:
			self._command_guard.record_failure(error)

	def _track_loop_rate(self, loop_start: float) -> None:
		"""루프 실측 주기를 집계하고 목표에 못 미치면 경고한다."""
		self._rate_window_iterations += 1
		window_s = loop_start - self._rate_window_start
		if window_s < LOOP_RATE_REPORT_INTERVAL_S:
			return
		actual_hz = self._rate_window_iterations / window_s
		if actual_hz < LOOP_RATE_WARN_RATIO * self.config.host.fps:
			logger.warning(
				f'Loop running at {actual_hz:.1f}Hz '
				f'(target {self.config.host.fps}Hz) - lower --host.fps, '
				'--host.jpeg_quality or camera resolution.'
			)
		self._rate_window_start = loop_start
		self._rate_window_iterations = 0

	def shutdown(self) -> None:
		"""홈포즈 복귀 → 토크 해제 → 소켓 정리 (1회만).

		SSH가 끊긴 뒤에도 실행될 수 있으므로 print 대신 logger를 쓴다 —
		닫힌 터미널에 print하면 OSError로 정리가 중단될 수 있다.
		"""
		if self._is_shutdown_done:
			return
		self._is_shutdown_done = True

		if (
			self.robot.is_connected
			and self.config.home_return.is_enabled
			and isinstance(self.robot, BiFollowerBase)
		):
			logger.info('Returning to home pose...')
			self.robot.return_to_home(self.config.home_return)
		try:
			logger.info('Disabling robot motor torque...')
			if self.robot.is_connected:
				self.robot.disconnect()
		except Exception as error:
			logger.warning(f'Error while disconnecting robot: {error}')

		self._cmd_socket.close(linger=0)
		self._observation_socket.close(linger=0)
		self._zmq_context.term()


@parser.wrap()
def serve(cfg: HostAppConfig) -> None:
	"""파싱된 설정으로 host를 실행한다.

	Args:
		cfg: CLI에서 파싱된 실행 설정.
	"""
	init_logging()
	# setup()(캘리브레이션 대기·토크 ON) 중에도 SIGTERM/SSH 끊김이 정리
	# 경로를 타도록 로봇을 만들기 전에 등록한다 (메인 스레드).
	signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
	signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)
	host = RobotHost(cfg)
	# RobotHost가 --camera_head.mode로 has_head_motors를 확정한 뒤에
	# 찍어야 로그의 설정값이 실제 적용값과 같다.
	logging.info(pformat(asdict(cfg)))
	host.setup().run()


def main() -> None:
	"""서드파티 플러그인 등록 후 host를 시작한다."""
	register_third_party_plugins()
	try:
		serve()
	except KeyboardInterrupt:
		logger.info('Shutting down.')


if __name__ == '__main__':
	main()
