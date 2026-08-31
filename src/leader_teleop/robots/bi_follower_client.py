"""
원격 host에 붙는 양팔 팔로워 클라이언트
========================================
라즈베리파이에서 host.py가 실물 로봇(팔 2 + 카메라)을 열고 ZMQ로 관측을
내보내면, PC의 이 클라이언트가 그것을 lerobot Robot 인터페이스로
감쌉니다. 기존 teleoperate/record 경로는 `--robot.type=bi_so101_client`
로 로봇 타입만 바꾸면 그대로 동작합니다 (리더암은 PC에 직접 연결).

관측/액션 키는 짝이 되는 팔로워 클래스(follower_class)의 관절 명세에서
파생하므로 host 쪽 로봇과 항상 같은 키를 냅니다. 카메라는 PC가 열지
않지만 데이터셋 feature(해상도)를 연결 전에 알아야 하므로 host와 같은
`--robot.cameras`를 지정합니다 — `index_or_path`는 무시됩니다.

수신은 별도 스레드가 담당합니다. 소켓을 상시 비우며 **도착 시각**을
기록하므로, 큐에 묵혀 있던 메시지를 뒤늦게 읽어 신선도를 오판하는 일이
없고, 에피소드 저장처럼 제어 루프가 잠시 멈추는 동안에도 큐가 쌓이지
않습니다. JPEG 디코딩도 이 스레드에서 끝나 제어 루프를 막지 않습니다.

신선도 보호: host 관측이 `stale_timeout_s` 이상 끊기면 get_observation과
send_action이 DeviceNotConnectedError를 던져 루프를 멈춥니다. 멈춘 host의
오래된 프레임이 데이터셋에 조용히 기록되거나, 응답 없는 host에 명령만
계속 보내는 상황을 막기 위해서입니다.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from functools import cached_property
from typing import Any, ClassVar, Final

import numpy as np

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
from lerobot.utils.errors import (
	DeviceAlreadyConnectedError,
	DeviceNotConnectedError,
)

from ..remote_codec import (
	DEFAULT_PORT_ZMQ_CMD,
	DEFAULT_PORT_ZMQ_OBSERVATIONS,
	decode_observation,
)
from .bi_follower_base import BiFollowerBase
from .bi_so101_follower import BiSo101Follower
from .bi_so102_follower import BiSo102Follower

# pyzmq는 원격 클라이언트를 실제로 쓸 때만 필요하다. 여기서 실패시키면
# 직접 연결(bi_so101_follower)만 쓰는 환경까지 패키지 임포트가 깨지므로,
# 없으면 클라이언트 생성 시점에 안내와 함께 실패한다.
try:
	import zmq
except ImportError:  # pragma: no cover - 환경 의존
	zmq = None  # type: ignore[assignment]


logger = logging.getLogger(__name__)

# 수신 스레드의 poll 간격 (ms). 종료 요청 반응 시간의 상한이기도 하다.
RECEIVE_POLL_MS: Final[int] = 50
# 종료 시 수신 스레드 join 대기 상한 (s). 정상이면 poll 1회 안에 끝난다.
RECEIVER_JOIN_TIMEOUT_S: Final[float] = 1.0
# 연결 중 캐시가 채워지길 기다릴 때의 확인 간격 (s).
CACHE_WAIT_POLL_S: Final[float] = 0.01


@dataclass
class BiFollowerClientConfig(RobotConfig):
	"""원격 양팔 팔로워 클라이언트 공통 설정 (타입 등록은 서브클래스)."""

	remote_ip: str  # host(라즈베리파이) IP
	port_zmq_cmd: int = DEFAULT_PORT_ZMQ_CMD  # 명령 송신 포트
	port_zmq_observations: int = DEFAULT_PORT_ZMQ_OBSERVATIONS  # 관측 수신 포트
	# host와 같은 카메라 이름·해상도 (feature 계산용, index는 무시)
	cameras: dict[str, CameraConfig] = field(default_factory=dict)
	# 머리 모터 장착 여부. 조립 계층(app/record)이 --camera_head.mode로
	# 설정한다 - host 쪽 로봇과 같아야 관측/액션 키가 일치한다.
	has_head_motors: bool = False
	connect_timeout_s: float = 5.0  # 첫 관측 대기 시간 (host 미실행 감지)
	stale_timeout_s: float = 1.0  # 관측이 이 시간 이상 끊기면 오류로 중단


class BiFollowerClient(Robot):
	"""ZMQ 너머의 양팔 팔로워를 lerobot Robot으로 노출하는 클라이언트.

	서브클래스가 정의해야 하는 것:

	Attributes:
		follower_class: 짝이 되는 실물 팔로워 클래스. 관절 명세
			(arm_joint_specs)를 빌려 관측/액션 키를 만든다.
	"""

	follower_class: ClassVar[type[BiFollowerBase]]

	def __init__(self, config: BiFollowerClientConfig) -> None:
		if zmq is None:
			raise ImportError(
				'pyzmq is required for the remote client '
				f'({config.type}). Install it with: pip install '
				'pyzmq==27.1.0  (included in requirements.txt)'
			)
		super().__init__(config)
		self.config = config
		# record.py가 이미지 라이터 스레드 수를 len(robot.cameras)로
		# 정하므로 카메라 이름만 담은 딕셔너리를 같은 이름으로 둔다.
		self.cameras: dict[str, None] = dict.fromkeys(config.cameras)
		self._zmq_context: zmq.Context | None = None
		self._cmd_socket: zmq.Socket | None = None
		self._observation_socket: zmq.Socket | None = None
		# 아래 캐시는 수신 스레드가 쓰고 제어 루프가 읽는다 (_lock으로 보호).
		self._lock = threading.Lock()
		self._last_state: dict[str, float] = {}
		self._last_frames: dict[str, np.ndarray] = {}
		self._last_receive_time: float | None = None  # 도착 시각 (monotonic)
		self._receiver_thread: threading.Thread | None = None
		self._stop_event = threading.Event()
		self._is_connected = False
		self._has_warned_missing_keys = False
		self._has_warned_command_drop = False
		self._has_warned_receive_error = False

	# ------------------------------------------------------------
	# 관측 / 명령 피처 (host 쪽 BiFollowerBase와 같은 규칙)
	# ------------------------------------------------------------
	@property
	def _state_ft(self) -> dict[str, type]:
		"""양팔 관절 (+머리 장착 시 2축) 상태 피처."""
		joint_names = [
			joint for joint, _ in self.follower_class.arm_joint_specs
		] + ['gripper']
		state_keys = [
			f'{prefix}_{joint}.pos'
			for prefix in ('left_arm', 'right_arm')
			for joint in joint_names
		]
		if self.config.has_head_motors:
			state_keys += ['head_motor_1.pos', 'head_motor_2.pos']
		return dict.fromkeys(state_keys, float)

	@property
	def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
		return {
			name: (camera.height, camera.width, 3)
			for name, camera in self.config.cameras.items()
		}

	@cached_property
	def observation_features(self) -> dict[str, type | tuple]:
		"""관측 피처 (관절 상태 + 카메라 shape). 연결 전에도 정확하다."""
		return {**self._state_ft, **self._cameras_ft}

	@cached_property
	def action_features(self) -> dict[str, type]:
		"""액션 피처 (관절 상태 키와 동일)."""
		return self._state_ft

	@property
	def is_connected(self) -> bool:
		return self._is_connected

	@property
	def is_calibrated(self) -> bool:
		"""캘리브레이션은 host 쪽 로봇이 담당한다."""
		return True

	# ------------------------------------------------------------
	# 연결 / 해제
	# ------------------------------------------------------------
	def connect(self, calibrate: bool = True) -> None:
		"""host에 ZMQ 소켓을 연결하고 첫 관측으로 구성을 검증한다.

		첫 관측에서 선언한 카메라가 모두 오는지, 프레임 해상도가 설정과
		같은지 확인한다 — 여기서 잡지 않으면 에피소드 도중 데이터셋
		검증에서 죽는다.

		Args:
			calibrate: 인터페이스 호환용 (client는 캘리브레이션하지 않음).

		Raises:
			DeviceAlreadyConnectedError: 이미 연결된 경우.
			DeviceNotConnectedError: connect_timeout_s 안에 관측이 오지
				않거나(host 미실행, IP 오류, 방화벽), 카메라 구성이
				host와 다를 때.
		"""
		if self._is_connected:
			raise DeviceAlreadyConnectedError(f'{self} already connected')

		# 재연결 시 이전 세션 캐시로 검증이 통과하지 않게 비운다.
		with self._lock:
			self._last_state = {}
			self._last_frames = {}
			self._last_receive_time = None

		self._zmq_context = zmq.Context()
		self._cmd_socket = self._zmq_context.socket(zmq.PUSH)
		self._cmd_socket.setsockopt(zmq.CONFLATE, 1)
		# IMMEDIATE=1: 연결이 완료된 상대에게만 큐잉한다. host가 없으면
		# 송신이 zmq.Again으로 실패해 "명령 드롭"을 감지할 수 있다.
		self._cmd_socket.setsockopt(zmq.IMMEDIATE, 1)
		self._cmd_socket.connect(
			f'tcp://{self.config.remote_ip}:{self.config.port_zmq_cmd}'
		)
		self._observation_socket = self._zmq_context.socket(zmq.PULL)
		self._observation_socket.setsockopt(zmq.CONFLATE, 1)
		self._observation_socket.connect(
			f'tcp://{self.config.remote_ip}:'
			f'{self.config.port_zmq_observations}'
		)

		self._stop_event.clear()
		self._receiver_thread = threading.Thread(
			target=self._receive_loop, name='bi_follower_client_rx',
			daemon=True,
		)
		self._receiver_thread.start()
		try:
			self._wait_for_first_observation()
			self._validate_camera_frames()
		except BaseException:
			# 타임아웃뿐 아니라 대기 중 Ctrl+C에도 스레드·소켓을 정리한다.
			self._close_sockets()
			raise
		self._is_connected = True
		logger.info(f'{self} connected to host {self.config.remote_ip}.')

	def _wait_for_first_observation(self) -> None:
		"""connect_timeout_s 안에 수신 스레드가 첫 관측을 받을 때까지 기다린다.

		Raises:
			DeviceNotConnectedError: 시간 안에 관측이 오지 않을 때.
		"""
		deadline = time.monotonic() + self.config.connect_timeout_s
		while time.monotonic() < deadline:
			with self._lock:
				if self._last_receive_time is not None:
					return
			time.sleep(CACHE_WAIT_POLL_S)
		raise DeviceNotConnectedError(
			f'No observation from host {self.config.remote_ip} within '
			f'{self.config.connect_timeout_s}s - is host.py running on '
			'the Pi, and is the IP/port reachable?'
		)

	def _validate_camera_frames(self) -> None:
		"""첫 관측의 카메라 프레임이 설정과 맞는지 검사한다.

		인코딩 실패로 프레임이 빠진 한 장 때문에 오판하지 않도록, 빠진
		카메라가 있으면 잠시 더 받아본 뒤 판정한다.

		Raises:
			DeviceNotConnectedError: 선언한 카메라가 host에서 오지 않거나
				프레임 shape가 설정 해상도와 다를 때.
		"""
		deadline = time.monotonic() + self.config.connect_timeout_s
		while time.monotonic() < deadline:
			with self._lock:
				frames = dict(self._last_frames)
			if not set(self._cameras_ft) - set(frames):
				break
			time.sleep(CACHE_WAIT_POLL_S)

		for name, shape in self._cameras_ft.items():
			frame = frames.get(name)
			if frame is None:
				raise DeviceNotConnectedError(
					f'Host is not sending camera {name!r} - check that host '
					'--robot.cameras declares the same camera names.'
				)
			if tuple(frame.shape) != shape:
				raise DeviceNotConnectedError(
					f'Camera {name!r} frame shape {tuple(frame.shape)} != '
					f'configured {shape} - match width/height (and '
					'rotation) with the host --robot.cameras.'
				)

	def calibrate(self) -> None:
		"""host 쪽 로봇이 캘리브레이션을 담당하므로 할 일이 없다."""

	def configure(self) -> None:
		"""host 쪽 로봇이 모터 설정을 담당하므로 할 일이 없다."""

	def disconnect(self) -> None:
		"""ZMQ 소켓을 닫는다 (host 쪽 로봇은 그대로 유지된다).

		Raises:
			DeviceNotConnectedError: 연결되지 않은 경우.
		"""
		if not self._is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')
		self._close_sockets()
		self._is_connected = False
		logger.info(f'{self} disconnected.')

	def _close_sockets(self) -> None:
		"""수신 스레드를 멈춘 뒤 소켓과 컨텍스트를 정리한다.

		ZMQ 소켓은 스레드 안전하지 않으므로 스레드가 끝난 뒤에 닫는다.
		스레드가 제때 안 끝나면(비정상) 관측 소켓과 컨텍스트는 닫지 않고
		둔다 — 누수가 다른 스레드의 소켓을 닫는 미정의 동작보다 낫다.
		부분 연결 상태에서도 안전하다.
		"""
		self._stop_event.set()
		is_receiver_alive = False
		if self._receiver_thread is not None:
			self._receiver_thread.join(timeout=RECEIVER_JOIN_TIMEOUT_S)
			is_receiver_alive = self._receiver_thread.is_alive()
			self._receiver_thread = None
		if self._cmd_socket is not None:
			self._cmd_socket.close(linger=0)
			self._cmd_socket = None
		if is_receiver_alive:
			logger.warning(
				'Receiver thread did not stop in time - leaving the '
				'observation socket open to avoid closing it from another '
				'thread.'
			)
			return
		if self._observation_socket is not None:
			self._observation_socket.close(linger=0)
			self._observation_socket = None
		if self._zmq_context is not None:
			self._zmq_context.term()
			self._zmq_context = None

	# ------------------------------------------------------------
	# 관측 / 명령
	# ------------------------------------------------------------
	def _receive_loop(self) -> None:
		"""수신 스레드 본체: 소켓을 상시 비우고 최신 관측을 캐시에 반영한다.

		관측 소켓은 이 스레드만 만진다. 도착 즉시 디코딩하므로 캐시의
		수신 시각이 실제 도착 시각과 일치한다. 개별 메시지 처리 오류로는
		죽지 않고 다음 메시지로 넘어간다.
		"""
		socket = self._observation_socket
		poller = zmq.Poller()
		poller.register(socket, zmq.POLLIN)
		while not self._stop_event.is_set():
			try:
				if socket not in dict(poller.poll(RECEIVE_POLL_MS)):
					continue
				latest: str | None = None
				while True:
					try:
						latest = socket.recv_string(zmq.NOBLOCK)
					except zmq.Again:
						break
				if latest is not None:
					self._update_from_message(latest)
			except zmq.ZMQError as error:
				if not self._stop_event.is_set():
					logger.error(f'Observation socket error: {error}')
				return
			except Exception as error:
				if not self._has_warned_receive_error:
					self._has_warned_receive_error = True
					logger.error(
						f'Observation processing failed ({error}) - '
						'skipping this message.'
					)

	def _update_from_message(self, message: str) -> None:
		"""관측 메시지를 디코딩해 캐시와 도착 시각을 갱신한다."""
		try:
			decoded = decode_observation(message, self.config.cameras)
		except ValueError as error:
			logger.error(f'Malformed observation from host: {error}')
			return
		with self._lock:
			self._last_state = decoded.state
			self._last_frames.update(decoded.frames)
			self._last_receive_time = time.monotonic()

	def _raise_if_stale(self) -> None:
		"""마지막 관측 도착 후 stale_timeout_s가 지났으면 예외를 던진다.

		Raises:
			DeviceNotConnectedError: host 관측이 끊긴 경우.
		"""
		with self._lock:
			last_receive_time = self._last_receive_time
		if last_receive_time is None:
			return
		age_s = time.monotonic() - last_receive_time
		if age_s > self.config.stale_timeout_s:
			raise DeviceNotConnectedError(
				f'No observation from host for {age_s:.1f}s '
				f'(stale_timeout_s={self.config.stale_timeout_s}). Host '
				'stopped or network lost - stopping to avoid recording '
				'stale frames or commanding an unresponsive host.'
			)

	def get_observation(self) -> dict[str, Any]:
		"""수신 스레드가 받아 둔 최신 관측을 반환한다.

		Raises:
			DeviceNotConnectedError: 연결되지 않았거나, host 관측이
				stale_timeout_s 이상 끊겼을 때.
		"""
		if not self._is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')
		self._raise_if_stale()

		with self._lock:
			state = self._last_state
			frames = dict(self._last_frames)

		observation: dict[str, Any] = {}
		missing_keys = []
		for key in self._state_ft:
			if key in state:
				observation[key] = state[key]
			else:
				observation[key] = 0.0
				missing_keys.append(key)
		if missing_keys and not self._has_warned_missing_keys:
			self._has_warned_missing_keys = True
			logger.warning(
				f'Host observation is missing keys {missing_keys} - check '
				'that host and client use the same robot type and '
				'--camera_head.mode.'
			)
		for name in self._cameras_ft:
			observation[name] = frames[name]
		return observation

	def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
		"""관절 목표각을 host로 전송한다.

		유한하지 않은 값(NaN/inf)은 보내지 않는다. 이동량 제한은 host의
		`--robot.max_relative_target`이 담당하며, 기본값(None)이면 제한이
		없다.

		Args:
			action: '{모터}.pos' -> 목표각 딕셔너리 (다른 키는 무시).

		Returns:
			전송한 action 딕셔너리.

		Raises:
			DeviceNotConnectedError: 연결되지 않았거나, host 관측이
				stale_timeout_s 이상 끊겼을 때 (텔레옵 전용 경로도
				host 이탈 시 멈추게 한다).
		"""
		if not self._is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')
		self._raise_if_stale()

		goal_pos = {
			key: float(value)
			for key, value in action.items()
			if key.endswith('.pos') and math.isfinite(float(value))
		}
		try:
			self._cmd_socket.send_string(json.dumps(goal_pos), zmq.NOBLOCK)
			if self._has_warned_command_drop:
				self._has_warned_command_drop = False
				logger.info('Command delivery to host resumed.')
		except zmq.Again:
			if not self._has_warned_command_drop:
				self._has_warned_command_drop = True
				logger.warning('Command dropped - host not connected.')
		return goal_pos


@RobotConfig.register_subclass('bi_so101_client')
@dataclass
class BiSo101ClientConfig(BiFollowerClientConfig):
	"""SO-101 양팔 원격 클라이언트 설정 (공통 필드는 BiFollowerClientConfig)."""


class BiSo101Client(BiFollowerClient):
	"""host의 bi_so101_follower와 짝이 되는 클라이언트."""

	config_class = BiSo101ClientConfig
	name = 'bi_so101_client'
	follower_class = BiSo101Follower


@RobotConfig.register_subclass('bi_so102_client')
@dataclass
class BiSo102ClientConfig(BiFollowerClientConfig):
	"""SO-102 양팔 원격 클라이언트 설정 (공통 필드는 BiFollowerClientConfig)."""


class BiSo102Client(BiFollowerClient):
	"""host의 bi_so102_follower와 짝이 되는 클라이언트."""

	config_class = BiSo102ClientConfig
	name = 'bi_so102_client'
	follower_class = BiSo102Follower
