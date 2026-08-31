"""
텔레옵 애플리케이션 (오케스트레이션)
====================================
로봇/리더암/카메라 헤드 스택의 조립(TeleopApp.__init__)·연결(setup)·
정리(shutdown)를 담당합니다. 직접 실행(teleoperate.py)은 여기의 run()
루프를 쓰고, record 경로(record.py)도 **같은 TeleopApp으로 조립·연결·
정리**한 뒤 제어 루프만 lerobot record_loop으로 바꿉니다 — 두 경로의
차이는 에피소드 녹화 유무뿐입니다. CLI 체계도 동일합니다 (draccus
`--robot.*` / `--teleop.*` / `--camera_head.*` / `--home_return.*`).

텔레옵 입력은 물리 리더암(`--teleop.type=bi_so101_leader` /
`bi_so102_leader`, BiLeaderBase 계열)입니다. 리더암을 손으로 움직이면
팔로워암이 관절 단위로 따라갑니다. 리더암은 손을 놓으면 팔로워가 그
자리에 멈추는 구조라 별도의 데드맨 스위치가 없습니다.

카메라 헤드는 `--camera_head.mode`(none/fixed/keyboard)로 고르고,
CombinedTeleop이 팔 12키 + 머리 2키를 합쳐 robot.send_action() 한 번으로
보냅니다 (record 경로와 동일).

실행 순서 (record 경로와 동일):

	로봇 연결 -> 텔레옵 연결 -> 관측값 영점 초기화 -> 실시간 제어 루프
"""

import time
from dataclasses import dataclass, field

from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.teleoperators import TeleoperatorConfig
from lerobot.utils.robot_utils import precise_sleep

from .config import HomeReturnConfig
from .head.head_modes import HeadControlConfig, make_head_controller
from .robots import (
	BiFollowerBase,
	BiFollowerBaseConfig,
	BiFollowerClientConfig,
	BiSo101FollowerConfig,
)
from .teleoperators import BiSo101LeaderConfig
from .teleoperators.combined_teleop import (
	CombinedTeleop,
	assert_arm_features_match,
	build_arm_teleop,
)


@dataclass
class TeleopStackConfig:
	"""조립 계층(TeleopApp) 공통 부가 설정 묶음 (두 진입점이 상속).

	robot/teleop 필드는 각 진입점 config(TeleopAppConfig / lerobot
	RecordConfig)가 정의한다 - TeleopApp은 두 필드가 있다고 가정하고
	덕 타이핑으로 조립한다.
	"""

	# 카메라 헤드 (--camera_head.mode=none|fixed|keyboard)
	camera_head: HeadControlConfig = field(default_factory=HeadControlConfig)
	# 종료 시 홈포즈(수그린 자세) 복귀 (--home_return.*)
	home_return: HomeReturnConfig = field(default_factory=HomeReturnConfig)


@dataclass
class TeleopAppConfig(TeleopStackConfig):
	"""직접 실행(teleoperate.py) 설정 — 인자 체계는 record와 동일."""

	# 로봇 하드웨어 (--robot.type=bi_so101_follower 등). 기본: bi_so101_follower
	robot: RobotConfig = field(default_factory=BiSo101FollowerConfig)
	# 팔 텔레옵 (--teleop.type=bi_so101_leader|bi_so102_leader).
	# 기본: bi_so101_leader (포트 인자는 필수 - 없으면 시작 시 에러 안내)
	teleop: TeleoperatorConfig = field(default_factory=BiSo101LeaderConfig)
	# 제어 루프 주기 (Hz)
	fps: int = 50


class TeleopApp:
	"""양팔 리더암 텔레옵 + 카메라 헤드 전체 실행을 오케스트레이션한다.

	Attributes:
		config: 실행 설정 (TeleopAppConfig).
		robot: 텔레옵 대상 로봇 (기본 BiSo101Follower).
		arm_teleop: 팔 텔레오퍼레이터 (물리 리더암).
		teleop: 팔 + 카메라 헤드 합성 텔레오퍼레이터 (CombinedTeleop).
	"""

	def __init__(self, config: TeleopStackConfig) -> None:
		self.config = config
		# 머리 모터 장착 여부는 --camera_head.mode가 단일 출처다 -
		# none(기본)이면 미장착으로 보고 bus1이 팔 모터만 검색한다.
		# 원격 클라이언트도 같은 규칙으로 관측/액션 키를 맞춘다.
		if isinstance(
			config.robot, (BiFollowerBaseConfig, BiFollowerClientConfig)
		):
			config.robot.has_head_motors = (
				config.camera_head.mode.lower() != 'none'
			)
		self.robot = make_robot_from_config(config.robot)
		self.arm_teleop = build_arm_teleop(config.teleop)
		# 로봇/텔레옵 세대 혼용(예: SO102 로봇 + SO101 리더)을 연결 전에
		# 명확한 에러로 잡는다.
		assert_arm_features_match(self.robot, self.arm_teleop)
		head_controller = make_head_controller(config.camera_head)
		self.teleop = CombinedTeleop(self.arm_teleop, head_controller)
		self._is_shutdown_done = False

	# ------------------------------------------------------------
	def setup(self) -> 'TeleopApp':
		"""로봇/텔레옵/카메라 헤드를 연결·초기화한다 (체이닝용).

		리더암은 시작 홈 정렬을 하지 않는다 - 리더 자세를 따라가므로
		홈으로 옮겨도 첫 프레임에 리더 위치로 점프해 의미가 없다.

		Raises:
			BaseException: 초기화 도중 발생한 예외를 (연결된 경우 토크
				해제 후) 그대로 다시 던진다.
		"""
		try:
			print('[system] Connecting robot...')
			self.robot.connect()
			self.teleop.connect()

			# 로봇 실제 관절각으로 팔/머리 영점을 초기화한다 (단일 팔
			# 모드: 제어하지 않는 팔 hold 값 / 머리: 무점프 시작).
			self.teleop.initialize_from_observation(
				self.robot.get_observation()
			)
			return self
		except BaseException:
			# Ctrl+C(KeyboardInterrupt)는 BaseException을 상속하므로
			# except Exception으로는 setup 도중의 Ctrl+C를 못 잡습니다.
			print(
				'\n[system] Init interrupted/failed - '
				'disabling robot torque.'
			)
			self.shutdown()
			raise

	# ------------------------------------------------------------
	def run(self) -> None:
		"""실시간 제어 루프를 실행한다 (Ctrl+C로 종료, 종료 시 정리 보장).

		매 프레임 teleop.get_action()이 리더암 관절각을 읽어 팔 12키
		(+머리 2키)를 계산하고 robot.send_action()으로 전송한다.
		"""
		dt_nominal = 1.0 / self.config.fps
		try:
			print(
				'\n[system] Leader teleop running - move the '
				'leader arms. (Ctrl+C to stop)\n'
			)

			while True:
				loop_start = time.perf_counter()

				# 팔 12키 + 머리 2키를 합쳐 전송.
				action = self.teleop.get_action()
				self.robot.send_action(action)

				elapsed = time.perf_counter() - loop_start
				precise_sleep(max(0.0, dt_nominal - elapsed))

		except KeyboardInterrupt:
			print('\n\n[system] Shutting down safely.')
		finally:
			self.shutdown()

	# ------------------------------------------------------------
	def shutdown(self) -> None:
		"""홈포즈 복귀 후 로봇 토크 해제를 수행한다 (1회만).

		홈포즈 복귀는 best-effort(실패해도 무시)이며 어떤 경우에도 토크
		해제는 진행된다.
		"""
		if self._is_shutdown_done:
			return
		self._is_shutdown_done = True

		if (
			self.robot.is_connected
			and self.config.home_return.is_enabled
			and isinstance(self.robot, BiFollowerBase)
		):
			print('[system] Returning to home pose...')
			self.robot.return_to_home(self.config.home_return)

		try:
			print('[system] Disabling robot motor torque...')
			if self.robot.is_connected:
				self.robot.disconnect()
		except Exception as error:
			print(f'[warn] Error while disconnecting robot: {error}')

		try:
			if self.teleop.is_connected:
				self.teleop.disconnect()
		except Exception as error:
			print(f'[warn] Error while shutting down teleop: {error}')
