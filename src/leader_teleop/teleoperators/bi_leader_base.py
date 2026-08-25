"""
양팔 리더암 텔레오퍼레이터 공통 구현
====================================
물리 리더암을 손으로 움직여 팔로워암을 조종하는 방식. 한쪽 팔은
`_SingleArmLeader`(lerobot SOLeader 상속 — SOLeader는 6모터 고정이라
SO-102의 7모터 팔을 위해 모터 구성만 일반화)가 담당하고, 이 모듈의
`BiLeaderBase`가 좌우 팔을 묶어 우리 로봇 키 스킴(`left_arm_`/
`right_arm_`)으로 리맵한다.

`mode`('left'/'right'/'dual')로 제어할 팔을 고른다. left/right면 해당
리더암 1대만 연결하고, 제어하지 않는 팔은 시작 관절각
(`initialize_from_observation`으로 캡처)을 hold한다 - action은 항상
양팔 전체 키를 내 record 데이터셋 feature 정합이 유지된다.

관절 구성·모터 id는 리더 세대(SO-101/SO-102)마다 다르므로, 서브클래스가
클래스 속성(arm_joint_specs/gripper_motor_id)으로 선언한다 — 팔로워
(robots/bi_follower_base.py)와 같은 패턴이다.

캘리브레이션 파일은 lerobot BiSOLeader와 같은 id 체계
(`{id}_left`/`{id}_right`)를 쓰며, 디렉터리는 리더 타입 이름
(teleoperators/{name}/)을 따르므로 세대 간에 파일이 섞이지 않는다.
"""

import logging
from dataclasses import dataclass
from typing import ClassVar, Final

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from ..hardware_retry import connect_with_retry, log_bus_voltages


logger = logging.getLogger(__name__)

# 팔 좌우 순서 (action 키 순서도 이 순서를 따른다).
ARM_SIDES: Final[tuple[str, str]] = ('left', 'right')
# mode가 가질 수 있는 값.
VALID_MODES: Final[tuple[str, str, str]] = ('left', 'right', 'dual')


class _SingleArmLeader(SOLeader):
	"""한쪽 리더암 (lerobot SOLeader 상속 - 모터 구성만 일반화).

	connect/calibrate/configure/get_action 절차·프롬프트는 SOLeader
	그대로다 (모터-파일 캘리브레이션 불일치 감지 -> 파일 사용/수동
	선택, full-turn 관절 'wrist_roll'은 가동범위 기록 없이 0~4095 -
	SO-101/SO-102 두 세대 모두 해당 관절이 'wrist_roll'이라 상속 가능).
	리더암은 사람이 움직이므로 configure()가 토크를 끈 채 둔다.
	"""

	name = 'bi_leader_arm'

	def __init__(
		self,
		config: SOLeaderTeleopConfig,
		arm_joint_specs: tuple[tuple[str, int], ...],
		gripper_motor_id: int,
	) -> None:
		super().__init__(config)
		# 상속한 SOLeader.calibrate()는 full-turn 관절을 'wrist_roll'로
		# 고정 가정한다 - 다른 이름의 연속 회전 관절을 가진 세대가 오면
		# 조용한 오캘리브레이션이 되므로 여기서 명세를 검증한다.
		joint_names = tuple(joint for joint, _ in arm_joint_specs)
		if 'wrist_roll' not in joint_names:
			raise ValueError(
				"arm_joint_specs must include 'wrist_roll' - inherited "
				'SOLeader.calibrate() hardcodes it as the full-turn joint '
				f'(got {joint_names}).'
			)
		# SOLeader가 6모터 고정으로 만든 버스를 관절 명세 기반으로
		# 재구성한다 (버스 생성은 포트를 열지 않아 부작용이 없다).
		motors = {
			joint: Motor(motor_id, 'sts3215', MotorNormMode.DEGREES)
			for joint, motor_id in arm_joint_specs
		}
		motors['gripper'] = Motor(
			gripper_motor_id, 'sts3215', MotorNormMode.RANGE_0_100
		)
		self.bus = FeetechMotorsBus(
			port=config.port,
			motors=motors,
			calibration=self.calibration,
		)


@dataclass
class BiLeaderBaseConfig(TeleoperatorConfig):
	"""양팔 리더암 공통 설정 (타입 등록은 리더별 서브클래스가 함)."""

	# 제어할 팔 ('left'/'right'/'dual'). 제어하지 않는 팔은 시작 자세 hold
	mode: str = 'dual'
	# 왼쪽 리더암 시리얼 포트 (mode=left/dual에서 필수)
	left_arm_port: str | None = None
	# 오른쪽 리더암 시리얼 포트 (mode=right/dual에서 필수)
	right_arm_port: str | None = None

	def __post_init__(self) -> None:
		"""mode와 필요한 포트가 지정됐는지 검증한다.

		Raises:
			ValueError: mode가 잘못됐거나 해당 mode에 필요한 포트가 없을 때.
		"""
		if self.mode not in VALID_MODES:
			raise ValueError(
				f"mode must be one of {VALID_MODES}, got '{self.mode}'."
			)
		if self.mode in ('left', 'dual') and self.left_arm_port is None:
			raise ValueError(
				f"left_arm_port is required for mode='{self.mode}'."
			)
		if self.mode in ('right', 'dual') and self.right_arm_port is None:
			raise ValueError(
				f"right_arm_port is required for mode='{self.mode}'."
			)


class BiLeaderBase(Teleoperator):
	"""좌우 리더암을 묶어 우리 로봇 키 스킴으로 리맵하는 텔레오퍼레이터.

	서브클래스가 정의해야 하는 것:

	Attributes:
		arm_joint_specs: (관절명, 모터 id) 순서 튜플 — 그리퍼 제외.
			짝이 되는 팔로워의 arm_joint_specs와 동일해야 한다.
		gripper_motor_id: 그리퍼 모터 id.
	"""

	arm_joint_specs: ClassVar[tuple[tuple[str, int], ...]]
	gripper_motor_id: ClassVar[int]

	def __init__(self, config: BiLeaderBaseConfig) -> None:
		super().__init__(config)
		self.config = config
		# 제어(연결) 대상 팔만 만든다. 캘리브레이션 id는 lerobot
		# BiSOLeader와 동일한 '{id}_{side}' 체계, 디렉터리는 이 클래스의
		# 경로(teleoperators/{name}, --teleop.calibration_dir로 오버라이드
		# 가능)를 물려줘 리더 세대 간에 파일이 섞이지 않는다.
		self._arms: dict[str, _SingleArmLeader] = {}
		for side in ARM_SIDES:
			if config.mode not in (side, 'dual'):
				continue
			arm_config = SOLeaderTeleopConfig(
				id=f'{self.id}_{side}' if self.id else None,
				calibration_dir=self.calibration_dir,
				port=getattr(config, f'{side}_arm_port'),
				use_degrees=True,  # 관절값 deg 정규화 (팔로워와 동일 전제)
			)
			self._arms[side] = _SingleArmLeader(
				arm_config,
				arm_joint_specs=self.arm_joint_specs,
				gripper_motor_id=self.gripper_motor_id,
			)
		# 제어하지 않는 팔의 hold 값 (initialize_from_observation이 채움).
		self._held_action: dict[str, float] = {}

	def _build_held_keys(self, side: str) -> tuple[str, ...]:
		"""한쪽 팔의 hold 대상 action 키를 만든다 (관절 + 그리퍼).

		Args:
			side: 팔 좌우 ('left'/'right').

		Returns:
			`{side}_arm_{관절}.pos` 키 튜플.
		"""
		joint_names = [
			joint for joint, _ in self.arm_joint_specs
		] + ['gripper']
		return tuple(
			f'{side}_arm_{joint_name}.pos' for joint_name in joint_names
		)

	# ------------------------------------------------------------
	# lerobot Teleoperator 인터페이스
	# ------------------------------------------------------------
	@property
	def action_features(self) -> dict[str, type]:
		features: dict[str, type] = {}
		for side in ARM_SIDES:
			if side in self._arms:
				features.update({
					f'{side}_arm_{key}': value_type
					for key, value_type
					in self._arms[side].action_features.items()
				})
			else:
				features.update(dict.fromkeys(
					self._build_held_keys(side), float
				))
		return features

	@property
	def feedback_features(self) -> dict[str, type]:
		return {}

	@property
	def is_connected(self) -> bool:
		return all(arm.is_connected for arm in self._arms.values())

	@property
	def is_calibrated(self) -> bool:
		return all(arm.is_calibrated for arm in self._arms.values())

	def connect(self, calibrate: bool = True) -> None:
		"""제어 대상 리더암을 연결한다 (간헐 실패 시 자동 재시도).

		간헐 통신 글리치('Incorrect status packet')와 일시적 모터 에러
		('Input voltage error')를 흡수한다. 실패한 시도는 포트를 닫고
		처음부터 다시 연결하며, 전압 에러 시 실측 전압을 진단 출력한다.
		"""
		for side, arm in self._arms.items():
			connect_with_retry(
				lambda arm=arm: arm.connect(calibrate),
				label=f'{side} leader arm connect',
				cleanup=lambda arm=arm: arm.bus.disconnect(
					disable_torque=False
				),
				diagnose=lambda side=side, arm=arm: log_bus_voltages(
					arm.bus, f'{side} leader'
				),
			)

	def calibrate(self) -> None:
		"""제어 대상 리더암을 캘리브레이션한다."""
		for arm in self._arms.values():
			arm.calibrate()

	def configure(self) -> None:
		"""제어 대상 리더암을 설정한다."""
		for arm in self._arms.values():
			arm.configure()

	def initialize_from_observation(
		self, observation: dict[str, float],
	) -> None:
		"""제어하지 않는 팔의 시작 관절각을 hold 값으로 캡처한다.

		dual이면 할 일이 없다. 단일 팔 모드에서는 이 메서드가 get_action
		전에 호출돼야 한다 (직접 실행 app.setup / record의
		CombinedTeleop.initialize_from_observation이 호출).

		Args:
			observation: 로봇 관측 딕셔너리 (관절 `.pos` 키 포함).

		Raises:
			KeyError: 관측에 hold 대상 관절 키가 없을 때.
		"""
		self._held_action = {}
		for side in ARM_SIDES:
			if side in self._arms:
				continue
			for key in self._build_held_keys(side):
				if key not in observation:
					raise KeyError(
						f"observation is missing '{key}' needed to hold "
						'the uncontrolled arm.'
					)
				self._held_action[key] = float(observation[key])

	def get_action(self) -> dict[str, float]:
		"""리더암 관절각(+hold 팔)을 우리 로봇 키로 반환한다.

		Returns:
			`{left_arm_*.pos, right_arm_*.pos}` 양팔 전체 키 딕셔너리.

		Raises:
			RuntimeError: 단일 팔 모드에서 hold 값이 초기화되지 않았을 때.
		"""
		action: dict[str, float] = {}
		for side, arm in self._arms.items():
			action.update({
				f'{side}_arm_{key}': value
				for key, value in arm.get_action().items()
			})
		if len(self._arms) < len(ARM_SIDES):
			if not self._held_action:
				raise RuntimeError(
					'Held-arm pose not initialized - call '
					'initialize_from_observation() before get_action().'
				)
			action.update(self._held_action)
		return action

	def send_feedback(self, feedback: dict[str, float]) -> None:
		"""리더암 피드백 미지원."""

	def disconnect(self) -> None:
		"""제어 대상 리더암을 연결 해제한다."""
		for arm in self._arms.values():
			arm.disconnect()
