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
"""

import json
import logging
from dataclasses import dataclass, field
from typing import Any, ClassVar

import numpy as np
import zmq

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


logger = logging.getLogger(__name__)


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
	polling_timeout_ms: int = 15  # 관측 폴링 대기 (없으면 직전 값 재사용)


class BiFollowerClient(Robot):
	"""ZMQ 너머의 양팔 팔로워를 lerobot Robot으로 노출하는 클라이언트.

	서브클래스가 정의해야 하는 것:

	Attributes:
		follower_class: 짝이 되는 실물 팔로워 클래스. 관절 명세
			(arm_joint_specs)를 빌려 관측/액션 키를 만든다.
	"""

	follower_class: ClassVar[type[BiFollowerBase]]

	def __init__(self, config: BiFollowerClientConfig) -> None:
		super().__init__(config)
		self.config = config
		# record.py가 이미지 라이터 스레드 수를 len(robot.cameras)로
		# 정하므로 카메라 이름만 담은 딕셔너리를 같은 이름으로 둔다.
		self.cameras: dict[str, None] = dict.fromkeys(config.cameras)
		self._zmq_context: zmq.Context | None = None
		self._cmd_socket: zmq.Socket | None = None
		self._observation_socket: zmq.Socket | None = None
		self._last_state: dict[str, float] = {}
		self._last_frames: dict[str, np.ndarray] = {}
		self._is_connected = False
		self._has_warned_missing_keys = False

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

	@property
	def observation_features(self) -> dict[str, type | tuple]:
		return {**self._state_ft, **self._cameras_ft}

	@property
	def action_features(self) -> dict[str, type]:
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
		"""host에 ZMQ 소켓을 연결하고 첫 관측이 올 때까지 기다린다.

		Args:
			calibrate: 인터페이스 호환용 (client는 캘리브레이션하지 않음).

		Raises:
			DeviceAlreadyConnectedError: 이미 연결된 경우.
			DeviceNotConnectedError: connect_timeout_s 안에 관측이 오지
				않을 때 (host 미실행, IP 오류, 방화벽).
		"""
		if self._is_connected:
			raise DeviceAlreadyConnectedError(f'{self} already connected')

		self._zmq_context = zmq.Context()
		self._cmd_socket = self._zmq_context.socket(zmq.PUSH)
		self._cmd_socket.setsockopt(zmq.CONFLATE, 1)
		self._cmd_socket.connect(
			f'tcp://{self.config.remote_ip}:{self.config.port_zmq_cmd}'
		)
		self._observation_socket = self._zmq_context.socket(zmq.PULL)
		self._observation_socket.setsockopt(zmq.CONFLATE, 1)
		self._observation_socket.connect(
			f'tcp://{self.config.remote_ip}:'
			f'{self.config.port_zmq_observations}'
		)

		first_message = self._poll_latest_message(
			int(self.config.connect_timeout_s * 1000)
		)
		if first_message is None:
			self._close_sockets()
			raise DeviceNotConnectedError(
				f'No observation from host {self.config.remote_ip} within '
				f'{self.config.connect_timeout_s}s - is host.py running on '
				'the Pi, and is the IP/port reachable?'
			)
		self._update_from_message(first_message)
		self._is_connected = True
		logger.info(f'{self} connected to host {self.config.remote_ip}.')

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
		"""소켓과 컨텍스트를 정리한다 (부분 연결 상태에서도 안전)."""
		for socket in (self._cmd_socket, self._observation_socket):
			if socket is not None:
				socket.close(linger=0)
		if self._zmq_context is not None:
			self._zmq_context.term()
		self._cmd_socket = None
		self._observation_socket = None
		self._zmq_context = None

	# ------------------------------------------------------------
	# 관측 / 명령
	# ------------------------------------------------------------
	def _poll_latest_message(self, timeout_ms: int) -> str | None:
		"""timeout_ms 안에 도착한 가장 최신 관측 메시지를 돌려준다."""
		poller = zmq.Poller()
		poller.register(self._observation_socket, zmq.POLLIN)
		if self._observation_socket not in dict(poller.poll(timeout_ms)):
			return None
		latest: str | None = None
		while True:
			try:
				latest = self._observation_socket.recv_string(zmq.NOBLOCK)
			except zmq.Again:
				return latest

	def _update_from_message(self, message: str) -> None:
		"""관측 메시지를 디코딩해 마지막 상태/프레임 캐시를 갱신한다."""
		try:
			state, frames = decode_observation(message, self.config.cameras)
		except json.JSONDecodeError as error:
			logger.error(f'Malformed observation from host: {error}')
			return
		self._last_state = state
		self._last_frames.update(frames)

	def get_observation(self) -> dict[str, Any]:
		"""최신 관측을 반환한다 (새 메시지가 없으면 직전 값 재사용).

		아직 못 받은 카메라 프레임은 설정 해상도의 검은 화면으로 채워
		데이터셋 feature와 항상 같은 형태를 유지한다.

		Raises:
			DeviceNotConnectedError: 연결되지 않은 경우.
		"""
		if not self._is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')

		message = self._poll_latest_message(self.config.polling_timeout_ms)
		if message is not None:
			self._update_from_message(message)

		observation: dict[str, Any] = {}
		missing_keys = []
		for key in self._state_ft:
			if key in self._last_state:
				observation[key] = self._last_state[key]
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
		for name, shape in self._cameras_ft.items():
			frame = self._last_frames.get(name)
			observation[name] = (
				frame if frame is not None
				else np.zeros(shape, dtype=np.uint8)
			)
		return observation

	def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
		"""관절 목표각을 host로 전송한다 (실제 클램프는 host가 수행).

		Args:
			action: '{모터}.pos' -> 목표각 딕셔너리 (다른 키는 무시).

		Returns:
			전송한 action 딕셔너리.

		Raises:
			DeviceNotConnectedError: 연결되지 않은 경우.
		"""
		if not self._is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')

		goal_pos = {
			key: float(value)
			for key, value in action.items()
			if key.endswith('.pos')
		}
		try:
			self._cmd_socket.send_string(json.dumps(goal_pos), zmq.NOBLOCK)
		except zmq.Again:
			logger.warning('Command dropped - host not receiving.')
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
