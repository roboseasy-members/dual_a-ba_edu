"""
양팔 + 머리 팔로워 공통 구현
============================
실기 lerobot의 Robot을 상속해 두 개의 Feetech 버스를 통합 관리합니다.

	- bus1 (left_arm_port): 왼팔 + 머리 2모터 (카메라 pan/tilt)
	- bus2 (right_arm_port): 오른팔

카메라 헤드 모터가 왼팔과 같은 버스(bus1)에 물려 있는 실제 배선과 그대로
일치하는 통합 버스 설계라, 머리 모터를 위해 같은 시리얼 포트를 이중으로
열거나 저수준 패킷으로 우회할 필요가 없습니다.

관절 구성·모터 id는 로봇 세대(SO-101/SO-102)마다 다르므로, 서브클래스가
클래스 속성(arm_joint_specs/gripper_motor_id/head_motor_ids)으로
선언합니다 — 로직은 전부 이 명세로부터 파생됩니다.
"""

import logging
import time
from dataclasses import dataclass, field
from functools import cached_property
from itertools import chain
from typing import Any, ClassVar

from lerobot.cameras import CameraConfig
# --robot.cameras의 type='opencv'는 이 설정 클래스가 임포트돼야 draccus
# 선택지로 등록된다 (lerobot record 스크립트와 같은 이유의 명시 임포트).
from lerobot.cameras.opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.errors import (
	DeviceAlreadyConnectedError,
	DeviceNotConnectedError,
)

from ..config import HomeReturnConfig
from ..hardware_retry import (
	BUS_WRITE_NUM_RETRY,
	connect_with_retry,
	log_bus_voltages,
)


logger = logging.getLogger(__name__)


@dataclass
class BiFollowerBaseConfig(RobotConfig):
	"""양팔+머리 팔로워 공통 설정 (타입 등록은 로봇별 서브클래스가 함)."""

	left_arm_port: str = '/dev/ttyACM0'  # bus1 포트 (왼팔 + 머리)
	right_arm_port: str = '/dev/ttyACM1'  # bus2 포트 (오른팔)
	disable_torque_on_disconnect: bool = True  # 해제 시 토크 자동 off
	max_relative_target: float | None = None  # 프레임당 이동 제한(안전)
	cameras: dict[str, CameraConfig] = field(default_factory=dict)  # 카메라
	# 머리(카메라 pan/tilt) 모터 장착 여부. 직접 지정하지 않는다 -
	# 조립 계층(app/record)이 --camera_head.mode(none이면 미장착)로
	# 설정한다. False면 bus1이 팔 모터만 검색/제어한다.
	has_head_motors: bool = True


class BiFollowerBase(Robot):
	"""왼팔+머리(bus1) / 오른팔(bus2)을 통합 관리하는 양팔 로봇.

	서브클래스가 정의해야 하는 것:

	Attributes:
		arm_joint_specs: (관절명, 모터 id) 순서 튜플 — 그리퍼 제외.
			관절명은 HomeReturnConfig의 '{관절명}_deg' 필드와 짝이어야
			한다 (홈포즈 복귀가 이 규칙으로 각도를 찾는다).
		gripper_motor_id: 그리퍼 모터 id (정규화 모드가 달라 분리).
		head_motor_ids: 머리 (pan id, tilt id).
	"""

	arm_joint_specs: ClassVar[tuple[tuple[str, int], ...]]
	gripper_motor_id: ClassVar[int]
	head_motor_ids: ClassVar[tuple[int, int]]

	def __init__(self, config: BiFollowerBaseConfig) -> None:
		super().__init__(config)
		self.config = config
		# 관절값은 deg 정규화 고정 - 리더암·헤드 각도 제어가 deg를 전제한다.
		norm_mode_body = MotorNormMode.DEGREES

		# bus1: 왼팔 + 머리. 머리 모터는 장착 선언(has_head_motors)이
		# 있을 때만 등록한다 - 미등록이면 연결 핸드셰이크·관측·제어
		# 전부에서 제외돼 미장착 구성에서도 정상 동작한다.
		bus1_motors = self._build_arm_motors('left_arm', norm_mode_body)
		if config.has_head_motors:
			pan_id, tilt_id = self.head_motor_ids
			bus1_motors['head_motor_1'] = Motor(
				pan_id, 'sts3215', norm_mode_body
			)
			bus1_motors['head_motor_2'] = Motor(
				tilt_id, 'sts3215', norm_mode_body
			)
		self.bus1 = FeetechMotorsBus(
			port=config.left_arm_port,
			motors=bus1_motors,
			calibration=self._bus_calibration(list(bus1_motors)),
		)

		# bus2: 오른팔.
		bus2_motors = self._build_arm_motors('right_arm', norm_mode_body)
		self.bus2 = FeetechMotorsBus(
			port=config.right_arm_port,
			motors=bus2_motors,
			calibration=self._bus_calibration(list(bus2_motors)),
		)

		self.left_arm_motors = [
			name for name in self.bus1.motors if name.startswith('left_arm')
		]
		self.right_arm_motors = [
			name for name in self.bus2.motors if name.startswith('right_arm')
		]
		self.head_motors = [
			name for name in self.bus1.motors if name.startswith('head')
		]
		self.cameras = make_cameras_from_configs(config.cameras)

		self._has_warned_dropped_keys = False

	def _build_arm_motors(
		self, prefix: str, norm_mode_body: MotorNormMode,
	) -> dict[str, Motor]:
		"""한쪽 팔의 모터 딕셔너리를 관절 명세로부터 만든다.

		Args:
			prefix: 'left_arm' 또는 'right_arm'.
			norm_mode_body: 관절 모터의 정규화 모드.

		Returns:
			모터 이름 -> Motor 딕셔너리 (관절 순서 + 그리퍼).
		"""
		motors = {
			f'{prefix}_{joint}': Motor(motor_id, 'sts3215', norm_mode_body)
			for joint, motor_id in self.arm_joint_specs
		}
		motors[f'{prefix}_gripper'] = Motor(
			self.gripper_motor_id, 'sts3215', MotorNormMode.RANGE_0_100
		)
		return motors

	def _bus_calibration(
		self, motor_names: list[str],
	) -> dict[str, MotorCalibration]:
		"""저장된 캘리브레이션 중 해당 버스 모터만 골라 반환한다.

		캘리브레이션 파일이 없으면(첫 실행) 빈 딕셔너리를 돌려주고,
		이후 connect()에서 수동 캘리브레이션을 유도합니다.

		Args:
			motor_names: 이 버스에 속한 모터 이름 목록.

		Returns:
			모터 이름 -> MotorCalibration 딕셔너리 (없으면 빈 딕셔너리).
		"""
		return {
			name: self.calibration[name]
			for name in motor_names
			if name in self.calibration
		}

	# ------------------------------------------------------------
	# 관측 / 명령 피처
	# ------------------------------------------------------------
	@property
	def _state_ft(self) -> dict[str, type]:
		"""양팔 관절 (+머리 장착 시 2축) 상태 피처."""
		joint_names = [
			joint for joint, _ in self.arm_joint_specs
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
	def _cameras_ft(self) -> dict[str, tuple]:
		return {
			cam: (
				self.config.cameras[cam].height,
				self.config.cameras[cam].width,
				3,
			)
			for cam in self.cameras
		}

	@cached_property
	def observation_features(self) -> dict[str, type | tuple]:
		return {**self._state_ft, **self._cameras_ft}

	@cached_property
	def action_features(self) -> dict[str, type]:
		return self._state_ft

	@property
	def is_connected(self) -> bool:
		return (
			self.bus1.is_connected
			and self.bus2.is_connected
			and all(cam.is_connected for cam in self.cameras.values())
		)

	@property
	def is_calibrated(self) -> bool:
		return self.bus1.is_calibrated and self.bus2.is_calibrated

	# ------------------------------------------------------------
	# 연결 / 캘리브레이션 / 설정
	# ------------------------------------------------------------
	def connect(self, calibrate: bool = True) -> None:
		"""두 버스와 카메라를 연결하고 모터를 설정한다.

		Args:
			calibrate: 저장된 캘리브레이션이 없을 때 수동 캘리브레이션을
				실행할지 여부.

		Raises:
			DeviceAlreadyConnectedError: 이미 연결된 경우.
		"""
		if self.is_connected:
			raise DeviceAlreadyConnectedError(f'{self} already connected')

		self.bus1.connect()
		self.bus2.connect()
		self._restore_or_calibrate(calibrate)

		for cam in self.cameras.values():
			cam.connect()

		# configure는 쓰기만 반복하므로 재호출이 안전하다 - 간헐 통신
		# 글리치/일시적 모터 에러로 한 번에 실패해도 재시도로 흡수한다.
		connect_with_retry(
			self.configure,
			label=f'{self} configure',
			diagnose=self._log_bus_voltages,
		)
		logger.info(f'{self} connected.')

	def _log_bus_voltages(self) -> None:
		"""두 버스의 모터 입력 전압을 진단 출력한다 (전압 에러 판정용)."""
		log_bus_voltages(self.bus1, 'bus1')
		log_bus_voltages(self.bus2, 'bus2')

	def _restore_or_calibrate(self, calibrate: bool) -> None:
		"""파일이 있으면 복원(모터에 write), 없으면 수동 캘리브레이션한다.

		파일을 직접 수정한 경우에도 복원을 선택하면 그 값이 그대로 모터에
		적용된다 (is_calibrated 자동 판정과 달리, 파일값을 모터에 다시
		써서 반영).

		복원 후 파일에 항목이 없는 모터(예: 팔만 캘리브레이션한 뒤 머리를
		새로 장착)가 있으면 그 모터만 부분 캘리브레이션해 파일에 병합한다
		— 이미 캘리브레이션된 팔은 다시 하지 않는다.

		Args:
			calibrate: 파일이 없거나 복원 실패 시 수동 캘리브레이션 실행 여부.
		"""
		if self.calibration_fpath.is_file():
			logger.info(f'Calibration file found at {self.calibration_fpath}')
			user_input = input(
				'Press ENTER to restore calibration from file, or type '
				"'c' and press ENTER to run manual calibration: "
			)
			if user_input.strip().lower() == 'c':
				if calibrate:
					self.calibrate()
				return
			try:
				self._write_calibration_from_file()
				logger.info('Calibration restored from file.')
			except Exception as error:
				logger.warning(f'Failed to restore calibration: {error}')
				if calibrate:
					self.calibrate()
				return
			if calibrate:
				self._calibrate_missing_motors()
		elif calibrate:
			logger.info('No calibration file found, running calibration...')
			self.calibrate()

	def _calibrate_missing_motors(self) -> None:
		"""파일에 항목이 없는 모터만 골라 부분 캘리브레이션한다.

		팔만 담긴 파일 상태에서 머리를 장착해 실행하면 머리 2모터만
		캘리브레이션하고 기존 파일에 병합 저장한다. 빠진 모터가 없으면
		아무것도 하지 않는다.
		"""
		for bus in (self.bus1, self.bus2):
			missing_motors = [
				name for name in bus.motors if name not in self.calibration
			]
			if missing_motors:
				self._calibrate_partial(bus, missing_motors)

	def _calibrate_partial(
		self, bus: FeetechMotorsBus, motor_names: list[str],
	) -> None:
		"""지정 모터만 캘리브레이션해 기존 캘리브레이션에 병합 저장한다.

		버스 캐시는 write_calibration이 통째로 교체하므로 기존 항목과
		병합한 딕셔너리를 넘긴다.

		Args:
			bus: 대상 모터가 속한 버스.
			motor_names: 캘리브레이션할 모터 이름 목록.
		"""
		joined_names = ', '.join(motor_names)
		logger.info(
			f'Calibrating only missing motors ({joined_names}) - '
			'already-calibrated motors keep their existing values.'
		)
		partial_calibration = self._run_bus_calibration(
			bus, motor_names, f'[{joined_names}]'
		)
		bus.write_calibration({**bus.calibration, **partial_calibration})
		self.calibration = {**self.calibration, **partial_calibration}
		self._save_calibration()
		print('Calibration updated:', self.calibration_fpath)

	def _write_calibration_from_file(self) -> None:
		"""파일 로드값(self.calibration)을 각 버스 모터에 써서 복원한다."""
		self._load_calibration()
		bus1_cal = {
			name: cal
			for name, cal in self.calibration.items()
			if name in self.bus1.motors
		}
		bus2_cal = {
			name: cal
			for name, cal in self.calibration.items()
			if name in self.bus2.motors
		}
		self.bus1.calibration = bus1_cal
		self.bus2.calibration = bus2_cal
		self.bus1.write_calibration(bus1_cal)
		self.bus2.write_calibration(bus2_cal)

	def _run_bus_calibration(
		self, bus: FeetechMotorsBus, motor_names: list[str], label: str,
	) -> dict[str, MotorCalibration]:
		"""한 버스의 지정 모터에 수동 캘리브레이션 절차를 수행한다.

		토크 해제 -> 위치 모드 -> 중립 자세 homing -> 가동범위 기록의
		공통 절차. 결과 저장(write_calibration/파일)은 호출측 책임이다.

		Args:
			bus: 대상 버스.
			motor_names: 캘리브레이션할 모터 이름 목록.
			label: 프롬프트에 표시할 대상 설명 (영어).

		Returns:
			모터 이름 -> MotorCalibration 딕셔너리.
		"""
		bus.disable_torque(motor_names)
		for name in motor_names:
			bus.write(
				'Operating_Mode', name, OperatingMode.POSITION.value
			)
		input(
			f'Move {label} to the middle of their range of motion and '
			'press ENTER....'
		)
		homing_offsets = bus.set_half_turn_homings(motor_names)
		print(
			f'Move {label} sequentially through their entire ranges of '
			'motion.\n'
			'Recording positions. Press ENTER to stop...'
		)
		range_mins, range_maxes = bus.record_ranges_of_motion(motor_names)
		return {
			name: MotorCalibration(
				id=bus.motors[name].id,
				drive_mode=0,
				homing_offset=homing_offsets[name],
				range_min=range_mins[name],
				range_max=range_maxes[name],
			)
			for name in motor_names
		}

	def calibrate(self) -> None:
		"""양팔 + 머리 모터를 수동 캘리브레이션한다."""
		logger.info(f'\nRunning calibration of {self}')

		calibration1 = self._run_bus_calibration(
			self.bus1,
			self.left_arm_motors + self.head_motors,
			'left arm and head motors',
		)
		self.bus1.write_calibration(calibration1)

		calibration2 = self._run_bus_calibration(
			self.bus2, self.right_arm_motors, 'right arm motors'
		)
		self.bus2.write_calibration(calibration2)

		self.calibration = {**calibration1, **calibration2}
		self._save_calibration()
		print('Calibration saved to', self.calibration_fpath)

	def configure(self) -> None:
		"""팔/머리 모터를 위치 제어 모드로 설정한다.

		개별 write에 num_retry를 줘 간헐 통신 글리치('Incorrect status
		packet')를 하위 계층에서 먼저 흡수한다 (upstream PR #4126 패턴).
		"""
		bus_motor_groups = (
			(self.bus1, self.left_arm_motors + self.head_motors),
			(self.bus2, self.right_arm_motors),
		)
		# 두 버스의 토크를 먼저 모두 끈다 - 한쪽 버스의 레지스터 쓰기가
		# 반복 실패해도 반대쪽 팔이 토크 켠 채 남지 않게 한다.
		for bus, _ in bus_motor_groups:
			bus.disable_torque(num_retry=BUS_WRITE_NUM_RETRY)
			bus.configure_motors()
		for bus, motor_names in bus_motor_groups:
			for name in motor_names:
				for register, value in (
					('Operating_Mode', OperatingMode.POSITION.value),
					('P_Coefficient', 16),
					('I_Coefficient', 0),
					('D_Coefficient', 43),
				):
					bus.write(
						register, name, value,
						num_retry=BUS_WRITE_NUM_RETRY,
					)
		self.bus1.enable_torque(num_retry=BUS_WRITE_NUM_RETRY)
		self.bus2.enable_torque(num_retry=BUS_WRITE_NUM_RETRY)

	def setup_motors(self) -> None:
		"""모터 id를 하나씩 설정한다 (초기 배선 시 1회)."""
		for motor in chain(
			reversed(self.left_arm_motors), reversed(self.head_motors)
		):
			input(
				f"Connect the controller board to the '{motor}' motor "
				'only and press ENTER.'
			)
			self.bus1.setup_motor(motor)
			print(f"'{motor}' motor id set to {self.bus1.motors[motor].id}")

		for motor in reversed(self.right_arm_motors):
			input(
				f"Connect the controller board to the '{motor}' motor "
				'only and press ENTER.'
			)
			self.bus2.setup_motor(motor)
			print(f"'{motor}' motor id set to {self.bus2.motors[motor].id}")

	# ------------------------------------------------------------
	# 관측 / 명령
	# ------------------------------------------------------------
	def get_observation(self) -> dict[str, Any]:
		"""양팔 + 머리 관절 위치와 카메라 프레임을 읽어 반환한다.

		Raises:
			DeviceNotConnectedError: 연결되지 않은 경우.
		"""
		if not self.is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')

		left_arm_pos = self.bus1.sync_read(
			'Present_Position', self.left_arm_motors
		)
		right_arm_pos = self.bus2.sync_read(
			'Present_Position', self.right_arm_motors
		)
		# 주의: lerobot sync_read는 빈 모터 목록에서 StopIteration으로
		# 죽는다 (모델 조회) - 머리 미장착이면 호출 자체를 건너뛴다.
		head_pos = (
			self.bus1.sync_read('Present_Position', self.head_motors)
			if self.head_motors else {}
		)

		obs_dict = {
			**{f'{k}.pos': v for k, v in left_arm_pos.items()},
			**{f'{k}.pos': v for k, v in right_arm_pos.items()},
			**{f'{k}.pos': v for k, v in head_pos.items()},
		}
		for cam_key, cam in self.cameras.items():
			obs_dict[cam_key] = cam.async_read()
		return obs_dict

	def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
		"""팔/머리 관절 목표각을 로봇으로 전송한다.

		max_relative_target이 설정돼 있으면 현재 위치 대비 이동량을
		안전하게 제한합니다.

		Args:
			action: prefix 포함 관절 키(...pos) -> 목표각 딕셔너리.

		Returns:
			실제로 전송된 action (클램프됐을 수 있음).

		Raises:
			DeviceNotConnectedError: 연결되지 않은 경우.
		"""
		if not self.is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')

		# 두 버스에 실재하는 모터의 '.pos' 키만 채택한다 (미장착 머리 키
		# 같은 나머지 키는 무시하되, 조용한 미구동을 막게 1회 경고).
		known_motors = self.bus1.motors.keys() | self.bus2.motors.keys()
		goal_pos = {
			key: value
			for key, value in action.items()
			if key.endswith('.pos')
			and key.removesuffix('.pos') in known_motors
		}
		if len(goal_pos) < sum(
			1 for key in action if key.endswith('.pos')
		) and not self._has_warned_dropped_keys:
			self._has_warned_dropped_keys = True
			dropped = sorted(
				key for key in action
				if key.endswith('.pos')
				and key.removesuffix('.pos') not in known_motors
			)
			logger.warning(
				f'send_action: dropping unknown motor keys {dropped} '
				'(motor not registered on either bus - check '
				'has_head_motors / joint names).'
			)

		if self.config.max_relative_target is not None:
			present_bus1 = self.bus1.sync_read(
				'Present_Position', self.left_arm_motors + self.head_motors
			)
			present_bus2 = self.bus2.sync_read(
				'Present_Position', self.right_arm_motors
			)
			present_pos = {**present_bus1, **present_bus2}
			goal_present_pos = {
				key: (g_pos, present_pos[key.removesuffix('.pos')])
				for key, g_pos in goal_pos.items()
			}
			goal_pos = ensure_safe_goal_position(
				goal_present_pos, self.config.max_relative_target
			)

		# 모터 소속 버스별로 한 번씩 sync_write한다 (bus1: 왼팔+머리).
		bus1_goal: dict[str, float] = {}
		bus2_goal: dict[str, float] = {}
		for key, value in goal_pos.items():
			name = key.removesuffix('.pos')
			target_goal = (
				bus1_goal if name in self.bus1.motors else bus2_goal
			)
			target_goal[name] = value
		if bus1_goal:
			self.bus1.sync_write('Goal_Position', bus1_goal)
		if bus2_goal:
			self.bus2.sync_write('Goal_Position', bus2_goal)

		return dict(goal_pos)

	def return_to_home(self, home_config: HomeReturnConfig) -> None:
		"""양팔/머리를 홈포즈(수그린 자세)로 부드럽게 복귀시킨다.

		duration_s 동안 현재 자세에서 홈포즈까지 선형 보간하며
		send_action()으로 전송한다 (max_relative_target 제한 존중).
		어떤 오류가 나도 예외를 던지지 않아 이후의 토크 해제(disconnect)를
		막지 않는다 (best-effort).

		Args:
			home_config: 홈포즈 관절각과 보간 시간 설정.
		"""
		try:
			# 홈 각도 필드 규칙: 관절 '{joint}'의 홈 각도는
			# HomeReturnConfig의 '{joint}_deg' 필드다 (그리퍼는 gripper_pos).
			arm_home = {
				joint: getattr(home_config, f'{joint}_deg')
				for joint, _ in self.arm_joint_specs
			}
			arm_home['gripper'] = home_config.gripper_pos
			target = {
				f'{prefix}_{joint}.pos': value
				for prefix in ('left_arm', 'right_arm')
				for joint, value in arm_home.items()
			}
			if self.head_motors:
				target['head_motor_1.pos'] = home_config.head_pan_deg
				target['head_motor_2.pos'] = home_config.head_tilt_deg

			present_bus1 = self.bus1.sync_read(
				'Present_Position', self.left_arm_motors + self.head_motors
			)
			present_bus2 = self.bus2.sync_read(
				'Present_Position', self.right_arm_motors
			)
			current = {
				f'{name}.pos': value
				for name, value in chain(
					present_bus1.items(), present_bus2.items()
				)
			}

			logger.info(
				'Returning to home pose '
				f'({home_config.duration_s:.1f}s interpolation)...'
			)
			steps = max(int(home_config.duration_s * home_config.fps), 1)
			step_dt = 1.0 / home_config.fps
			for step in range(1, steps + 1):
				ratio = step / steps
				action = {
					key: current[key] * (1.0 - ratio) + target[key] * ratio
					for key in target
				}
				self.send_action(action)
				time.sleep(step_dt)
			time.sleep(home_config.settle_s)
			logger.info('Home pose return complete.')
		except Exception as error:
			logger.warning(
				f'Home pose return failed - proceeding to torque off: {error}'
			)

	def disconnect(self) -> None:
		"""두 버스와 카메라 연결을 해제한다 (토크 자동 해제).

		Raises:
			DeviceNotConnectedError: 연결되지 않은 경우.
		"""
		if not self.is_connected:
			raise DeviceNotConnectedError(f'{self} is not connected.')

		self.bus1.disconnect(self.config.disable_torque_on_disconnect)
		self.bus2.disconnect(self.config.disable_torque_on_disconnect)
		for cam in self.cameras.values():
			cam.disconnect()
		logger.info(f'{self} disconnected.')
