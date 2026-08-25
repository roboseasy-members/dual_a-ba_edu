"""
합성 텔레오퍼레이터 (팔 + 카메라 헤드)
======================================
팔 텔레옵(리더암)과 카메라 헤드 컨트롤러를 하나의 lerobot
`Teleoperator`로 합친다. 직접 실행(app)과 record 두 경로 모두 매 사이클
`get_action()`을 부를 때 팔 12키 + 머리 2키 = **14키**를 반환한다.

우리 로봇은 머리 모터가 native라 저수준 버스 쓰기/래핑이 필요 없다.
머리 목표각을 action에 실어 보내면 record_loop의 `robot.send_action()`이
그대로 구동한다.

머리 컨트롤러가 없으면(mode=none, 머리 미장착 구성) 머리 키를 **아예
내지 않는다**(팔 12키만). 이때 로봇도 머리 모터를 등록하지 않으므로
데이터셋 action/observation과 rerun 표시에서 머리 2키가 함께 빠진다.
`get_action()`은 인자를 받지 않으므로 시간(t/dt)은 내부에서 추적한다.
"""

import time

from lerobot.robots.robot import Robot
from lerobot.teleoperators import (
	TeleoperatorConfig,
	make_teleoperator_from_config,
)
from lerobot.teleoperators.teleoperator import Teleoperator

from ..head.head_modes import PAN_MOTOR_KEY, TILT_MOTOR_KEY


def build_arm_teleop(teleop_config: TeleoperatorConfig) -> Teleoperator:
	"""텔레옵 설정 타입에 맞는 팔 텔레오퍼레이터를 만든다.

	직접 실행(app)과 record 두 경로가 공유하는 팩토리다. 리더암 타입은
	lerobot 팩토리에 위임한다 (teleoperators/__init__의 재노출로 탐색).

	Args:
		teleop_config: 파싱된 텔레옵 config (--teleop.type).

	Returns:
		lerobot Teleoperator 인스턴스.
	"""
	return make_teleoperator_from_config(teleop_config)


def assert_arm_features_match(robot: Robot, arm_teleop: Teleoperator) -> None:
	"""팔 텔레옵의 액션 키가 로봇의 팔 액션 키와 일치하는지 검증한다.

	로봇/리더 세대 혼용(예: bi_so102_follower + bi_so101_leader)을 연결
	전에 명확한 에러로 잡는다 - 그대로 두면 record feature 불일치나 일부
	관절 미구동 같은 조용한 오동작이 된다.

	Args:
		robot: 조립된 로봇 (action_features에 팔 + 머리 키 포함).
		arm_teleop: 조립된 팔 텔레오퍼레이터 (머리 키 없음).

	Raises:
		ValueError: 팔 키 집합이 일치하지 않을 때.
	"""
	robot_arm_keys = {
		key for key in robot.action_features
		if not key.startswith('head_')
	}
	teleop_keys = set(arm_teleop.action_features)
	if robot_arm_keys != teleop_keys:
		missing = sorted(robot_arm_keys - teleop_keys)
		extra = sorted(teleop_keys - robot_arm_keys)
		raise ValueError(
			'Arm teleop action keys do not match the robot '
			f'(missing={missing}, extra={extra}). Pair matching '
			'generations: bi_so101_follower with bi_so101_leader, '
			'bi_so102_follower with bi_so102_leader.'
		)


class CombinedTeleop(Teleoperator):
	"""팔 텔레옵 + 카메라 헤드를 합친 텔레오퍼레이터.

	자체 config가 없으므로 lerobot Teleoperator.__init__(캘리브레이션 경로
	생성 등)을 호출하지 않고, 필요한 속성만 직접 세팅한다.
	"""

	name = 'combined_teleop'

	def __init__(self, arm_teleop: Teleoperator, head_controller) -> None:
		self.arm = arm_teleop
		self.head = head_controller
		self.id = getattr(arm_teleop, 'id', None)
		self._last_t: float | None = None

	# ------------------------------------------------------------
	# lerobot Teleoperator 인터페이스 (팔에 위임 + 머리 2키 추가)
	# ------------------------------------------------------------
	@property
	def action_features(self) -> dict[str, type]:
		features = dict(self.arm.action_features)
		if self.head is not None:
			features[PAN_MOTOR_KEY] = float
			features[TILT_MOTOR_KEY] = float
		return features

	@property
	def feedback_features(self) -> dict[str, type]:
		return {}

	@property
	def is_connected(self) -> bool:
		return self.arm.is_connected

	@property
	def is_calibrated(self) -> bool:
		return self.arm.is_calibrated

	def connect(self, calibrate: bool = True) -> None:
		"""팔을 연결하고 머리 입력(키보드)을 시작한다."""
		self.arm.connect(calibrate)
		if self.head is not None:
			self.head.start()

	def calibrate(self) -> None:
		"""팔 캘리브레이션에 위임한다."""
		self.arm.calibrate()

	def configure(self) -> None:
		"""팔 설정에 위임한다."""
		self.arm.configure()

	def initialize_from_observation(
		self, observation: dict[str, float],
	) -> None:
		"""팔/머리 영점을 로봇 실제 관측값으로 초기화한다 (무점프 시작).

		Args:
			observation: 로봇 관측 딕셔너리 (14키 + 카메라).
		"""
		if hasattr(self.arm, 'initialize_from_observation'):
			self.arm.initialize_from_observation(observation)
		if self.head is not None:
			self.head.initialize_from_observation(observation)

	def get_action(self) -> dict[str, float]:
		"""팔 12키(+머리 컨트롤러가 있으면 머리 2키) action을 반환한다.

		Returns:
			머리 사용 시 14키, mode=none이면 팔 12키 딕셔너리.
		"""
		action = dict(self.arm.get_action())
		if self.head is None:
			return action

		now = time.perf_counter()
		dt = max(now - self._last_t, 1e-3) if self._last_t is not None else 1e-3
		self._last_t = now
		action.update(self.head.compute(now, dt))
		return action

	def send_feedback(self, feedback: dict[str, float]) -> None:
		"""팔에 위임한다 (Teleoperator 추상 메서드라 항상 존재)."""
		self.arm.send_feedback(feedback)

	def disconnect(self) -> None:
		"""머리 정리 후 팔을 연결 해제한다."""
		if self.head is not None:
			self.head.shutdown()
		self.arm.disconnect()
