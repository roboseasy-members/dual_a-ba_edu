"""
원격 host 진입점 (라즈베리파이에서 실행)
========================================
파이에 연결된 실물 양팔 팔로워(+카메라)를 열고, ZMQ로 관측을 내보내며
PC의 클라이언트(`--robot.type=bi_so101_client`)가 보내는 관절 명령을
받아 구동합니다. 리더암·키보드·데이터셋 저장은 전부 PC 쪽 몫이고,
host는 "로봇 드라이버를 네트워크로 연장"하는 역할만 합니다.

	PC (teleoperate/record + 리더암)  <--ZMQ-->  Pi (host + 팔로워·카메라)

CLI는 다른 진입점과 같은 draccus 체계입니다 (`--robot.*` /
`--camera_head.mode` / `--home_return.*`). 머리 모터 장착 여부는
`--camera_head.mode`(none이면 미장착)로 정하며, PC 쪽에서도 같은 값을
줘야 관측/액션 키가 일치합니다.

실행 예 (파이에서, 레포 루트):

	PYTHONPATH=src python -m leader_teleop.host \\
		--robot.type=bi_so101_follower --robot.id=bi_so101_follower \\
		--robot.left_arm_port=/dev/ttyACM0 \\
		--robot.right_arm_port=/dev/ttyACM1 \\
		--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}'

종료는 Ctrl+C. 홈포즈 복귀 후 토크가 풀립니다. PC 쪽 프로그램이 먼저
끝나도 host는 계속 돌며 팔을 마지막 자세로 유지합니다.
"""

import json
import logging
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

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


class RobotHost:
	"""실물 로봇을 ZMQ로 노출하는 서버 루프.

	Attributes:
		config: 실행 설정.
		robot: 이 파이에 연결된 실물 로봇.
	"""

	def __init__(self, config: HostAppConfig) -> None:
		self.config = config
		# 머리 장착 여부는 app과 같은 규칙(--camera_head.mode)으로 정한다.
		if isinstance(config.robot, BiFollowerBaseConfig):
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

	def setup(self) -> 'RobotHost':
		"""로봇을 연결하고 포트를 연다 (체이닝용)."""
		try:
			print('[host] Connecting robot...')
			self.robot.connect()
			self._cmd_socket.bind(f'tcp://*:{self.config.host.port_zmq_cmd}')
			self._observation_socket.bind(
				f'tcp://*:{self.config.host.port_zmq_observations}'
			)
			return self
		except BaseException:
			print('\n[host] Init interrupted/failed - disabling robot torque.')
			self.shutdown()
			raise

	def run(self) -> None:
		"""명령 수신 → 구동 → 관측 송신 루프 (Ctrl+C로 종료)."""
		camera_keys = tuple(self.robot.cameras)
		dt_nominal = 1.0 / self.config.host.fps
		has_warned_no_client = False
		try:
			print(
				f'\n[host] Serving on ports {self.config.host.port_zmq_cmd}'
				f'(cmd) / {self.config.host.port_zmq_observations}(obs). '
				'Waiting for client... (Ctrl+C to stop)\n'
			)
			while True:
				loop_start = time.perf_counter()

				try:
					message = self._cmd_socket.recv_string(zmq.NOBLOCK)
					self.robot.send_action(json.loads(message))
				except zmq.Again:
					pass  # 새 명령 없음 - 팔은 마지막 자세를 유지한다
				except Exception as error:
					logging.error(f'[host] Bad command dropped: {error}')

				observation = self.robot.get_observation()
				try:
					self._observation_socket.send_string(
						encode_observation(
							observation, camera_keys,
							self.config.host.jpeg_quality,
						),
						zmq.NOBLOCK,
					)
					has_warned_no_client = False
				except zmq.Again:
					if not has_warned_no_client:
						has_warned_no_client = True
						logging.info('[host] No client connected yet.')

				elapsed = time.perf_counter() - loop_start
				precise_sleep(max(0.0, dt_nominal - elapsed))
		except KeyboardInterrupt:
			print('\n\n[host] Shutting down safely.')
		finally:
			self.shutdown()

	def shutdown(self) -> None:
		"""홈포즈 복귀 → 토크 해제 → 소켓 정리 (1회만)."""
		if self._is_shutdown_done:
			return
		self._is_shutdown_done = True

		if (
			self.robot.is_connected
			and self.config.home_return.is_enabled
			and isinstance(self.robot, BiFollowerBase)
		):
			print('[host] Returning to home pose...')
			self.robot.return_to_home(self.config.home_return)
		try:
			print('[host] Disabling robot motor torque...')
			if self.robot.is_connected:
				self.robot.disconnect()
		except Exception as error:
			print(f'[warn] Error while disconnecting robot: {error}')

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
	logging.info(pformat(asdict(cfg)))
	RobotHost(cfg).setup().run()


def main() -> None:
	"""서드파티 플러그인 등록 후 host를 시작한다."""
	register_third_party_plugins()
	try:
		serve()
	except KeyboardInterrupt:
		print('\n[host] Shutting down.')


if __name__ == '__main__':
	main()
