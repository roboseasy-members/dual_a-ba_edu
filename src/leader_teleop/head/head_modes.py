"""
카메라(머리) 헤드 모드
======================
직접 실행/record 공통으로 카메라 pan/tilt 헤드를 어떻게 움직일지 3가지
모드로 제공합니다. 머리 모터(head_motor_1=pan, head_motor_2=tilt)는 로봇에
native로 통합돼 있으므로, 여기서는 **버스에 직접 쓰지 않고 목표각만
계산**해 반환합니다. 반환된 목표각은
CombinedTeleop이 로봇 action(`head_motor_1/2.pos`)에 실어 보내고,
record_loop의 `robot.send_action()`이 실제 구동합니다.

모드:
	none      - 머리 미장착 구성. 헤드 컨트롤러 없음 + 조립 계층이
		로봇의 머리 모터 검색 자체를 끈다(has_head_motors=False)
	fixed     - 시작 시 고정각(기본 홈각)으로 이동 후 토크 유지
	keyboard  - pynput wasd (a/d=pan, w/s=tilt, h=홈). record에선 방향키가
		에피소드 제어와 겹치므로 wasd 배치를 쓴다.

모든 컨트롤러의 공통 인터페이스:
	initialize_from_observation(obs) -> None
	start() -> None
	compute(t, dt) -> dict[str, float]   # {head_motor_1.pos, head_motor_2.pos}
	shutdown() -> None
"""

from dataclasses import dataclass
from typing import Final

import numpy as np

from ..config import HEAD_PAN_HOME_DEG, HEAD_TILT_HOME_DEG
from ..motion.trajectory import JointTrajectoryController


PAN_MOTOR_KEY: Final[str] = 'head_motor_1.pos'
TILT_MOTOR_KEY: Final[str] = 'head_motor_2.pos'


@dataclass
class HeadControlConfig:
	"""카메라 헤드 제어 설정 (--camera_head.*, draccus CLI 파싱 대상)."""

	mode: str = 'none'  # 'none'/'fixed'/'keyboard'
	pan_home_deg: float = HEAD_PAN_HOME_DEG  # pan 홈 각도 (deg)
	tilt_home_deg: float = HEAD_TILT_HOME_DEG  # tilt 홈 각도 (deg)
	pan_min_deg: float = -90.0  # pan 하한 (deg, 선 꼬임 방지)
	pan_max_deg: float = 90.0  # pan 상한 (deg)
	tilt_min_deg: float = -70.0  # tilt 하한 (deg)
	tilt_max_deg: float = 70.0  # tilt 상한 (deg)
	fixed_pan_deg: float | None = None  # fixed 모드 pan 목표 (None=홈)
	fixed_tilt_deg: float | None = None  # fixed 모드 tilt 목표 (None=홈)
	keyboard_speed_deg_s: float = 60.0  # keyboard 이동 속도 (deg/s)
	keyboard_pan_sign: float = 1.0  # keyboard a/d 방향 부호 (반대면 뒤집기)
	keyboard_tilt_sign: float = -1.0  # keyboard w/s 방향 부호
	traj_kp: float = 45.0  # fixed 궤적 반응성
	traj_max_vel_deg_s: float = 150.0  # 헤드 관절 속도 제한 (deg/s)
	traj_max_acc_deg_s2: float = 600.0  # 헤드 가속도 제한 (deg/s^2)


def _read_head_pose(
	observation: dict[str, float], config: HeadControlConfig,
) -> tuple[float, float]:
	"""관측에서 현재 (pan, tilt)를 읽는다 (키가 없으면 홈 각도)."""
	return (
		float(observation.get(PAN_MOTOR_KEY, config.pan_home_deg)),
		float(observation.get(TILT_MOTOR_KEY, config.tilt_home_deg)),
	)


class FixedHeadController:
	"""시작 시 고정각으로 이동 후 유지하는 헤드 컨트롤러."""

	def __init__(self, config: HeadControlConfig) -> None:
		self.config = config
		pan = (
			config.fixed_pan_deg
			if config.fixed_pan_deg is not None
			else config.pan_home_deg
		)
		tilt = (
			config.fixed_tilt_deg
			if config.fixed_tilt_deg is not None
			else config.tilt_home_deg
		)
		self._pan_target = float(np.clip(
			pan, config.pan_min_deg, config.pan_max_deg
		))
		self._tilt_target = float(np.clip(
			tilt, config.tilt_min_deg, config.tilt_max_deg
		))
		self._trajectory = JointTrajectoryController(
			kp=config.traj_kp,
			max_vel_deg_s=config.traj_max_vel_deg_s,
			max_acc_deg_s2=config.traj_max_acc_deg_s2,
		)

	def initialize_from_observation(
		self, observation: dict[str, float],
	) -> None:
		"""현재 머리 관절각으로 궤적을 초기화한다 (무점프 시작)."""
		self._trajectory.reset(
			np.array(_read_head_pose(observation, self.config))
		)

	def start(self) -> None:
		"""fixed 모드는 별도 입력 스레드가 없다."""

	def compute(self, t: float, dt: float) -> dict[str, float]:
		"""고정 목표각을 향해 한 스텝 진행한 값을 반환한다."""
		smoothed = self._trajectory.update(
			np.array([self._pan_target, self._tilt_target]), dt
		)
		return {
			PAN_MOTOR_KEY: float(smoothed[0]),
			TILT_MOTOR_KEY: float(smoothed[1]),
		}

	def shutdown(self) -> None:
		"""정리할 자원이 없다."""


class KeyboardHeadController:
	"""pynput wasd로 pan/tilt를 조작하는 헤드 컨트롤러.

	입력 스레드(pynput 리스너)는 눌린 키 집합만 갱신하고, 실제 각도 적분은
	메인 루프의 compute()에서 수행합니다 (버스 접근은 상위가 담당).
	"""

	def __init__(self, config: HeadControlConfig) -> None:
		self.config = config
		self._pan_deg = config.pan_home_deg
		self._tilt_deg = config.tilt_home_deg
		self._pressed: set[str] = set()
		self._listener = None

	def initialize_from_observation(
		self, observation: dict[str, float],
	) -> None:
		"""현재 머리 관절각을 시작값으로 삼는다 (무점프 시작)."""
		self._pan_deg, self._tilt_deg = _read_head_pose(
			observation, self.config
		)

	def start(self) -> None:
		"""pynput 전역 키 리스너를 시작한다.

		Raises:
			RuntimeError: pynput을 사용할 수 없는 경우.
		"""
		try:
			from pynput import keyboard
		except ImportError as error:
			raise RuntimeError(
				'keyboard head mode requires pynput. '
				'Install pynput or change --camera_head.mode.'
			) from error
		self._listener = keyboard.Listener(
			on_press=self._on_press, on_release=self._on_release
		)
		self._listener.start()

	def _key_char(self, key) -> str | None:
		return getattr(key, 'char', None)

	def _on_press(self, key) -> None:
		char = self._key_char(key)
		if char is not None:
			self._pressed.add(char.lower())

	def _on_release(self, key) -> None:
		char = self._key_char(key)
		if char is not None:
			self._pressed.discard(char.lower())

	def compute(self, t: float, dt: float) -> dict[str, float]:
		"""눌린 키 방향으로 목표각을 적분해 반환한다 (wasd + h=홈)."""
		if 'h' in self._pressed:
			self._pan_deg = self.config.pan_home_deg
			self._tilt_deg = self.config.tilt_home_deg

		pan_dir = ('d' in self._pressed) - ('a' in self._pressed)
		tilt_dir = ('w' in self._pressed) - ('s' in self._pressed)
		speed = self.config.keyboard_speed_deg_s

		self._pan_deg = float(np.clip(
			self._pan_deg
			+ self.config.keyboard_pan_sign * pan_dir * speed * dt,
			self.config.pan_min_deg,
			self.config.pan_max_deg,
		))
		self._tilt_deg = float(np.clip(
			self._tilt_deg
			+ self.config.keyboard_tilt_sign * tilt_dir * speed * dt,
			self.config.tilt_min_deg,
			self.config.tilt_max_deg,
		))
		return {
			PAN_MOTOR_KEY: self._pan_deg,
			TILT_MOTOR_KEY: self._tilt_deg,
		}

	def shutdown(self) -> None:
		"""키 리스너를 정지한다."""
		if self._listener is not None:
			self._listener.stop()
			self._listener = None


def make_head_controller(
	config: HeadControlConfig,
) -> FixedHeadController | KeyboardHeadController | None:
	"""모드에 맞는 헤드 컨트롤러를 생성한다 (none이면 None).

	Args:
		config: 카메라 헤드 설정.

	Returns:
		헤드 컨트롤러 또는 None(mode='none').

	Raises:
		ValueError: mode가 알 수 없는 값인 경우.
	"""
	mode = config.mode.lower()
	if mode == 'none':
		return None
	if mode == 'fixed':
		return FixedHeadController(config)
	if mode == 'keyboard':
		return KeyboardHeadController(config)
	raise ValueError(
		f'Unknown camera head mode: {mode!r} '
		"(must be one of 'none'/'fixed'/'keyboard')"
	)
