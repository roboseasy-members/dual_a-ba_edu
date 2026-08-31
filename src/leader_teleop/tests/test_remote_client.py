"""
BiFollowerClient 가짜 host 시나리오 테스트
==========================================
하드웨어 없이 루프백 ZMQ로 client의 연결·관측·명령·신선도·검증 경로를
확인한다. 수신 스레드/락 회귀를 잡는 것이 목적이다. pyzmq만 있으면 된다.

실행:
	PYTHONPATH=src python -m leader_teleop.tests.test_remote_client
	(pytest 호환: PYTHONPATH=src pytest src/leader_teleop/tests)
"""

import json
import threading
import time
from typing import Final

import numpy as np
import zmq

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.utils.errors import DeviceNotConnectedError

from leader_teleop.remote_codec import encode_observation
from leader_teleop.robots import BiSo101Client, BiSo101ClientConfig


# 다른 테스트·실서비스 포트(5555/5556)와 겹치지 않는 루프백 포트 대역.
BASE_PORT: Final[int] = 5800
JOINTS: Final[tuple[str, ...]] = (
	'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex',
	'wrist_roll', 'gripper',
)
HOST_HZ: Final[float] = 30.0


class _FakeHost(threading.Thread):
	"""host.py를 흉내 내는 스레드: 관측을 30Hz로 보내고 명령을 모은다."""

	def __init__(
		self, cmd_port: int, obs_port: int, cameras: dict[str, tuple[int, int]],
	) -> None:
		super().__init__(daemon=True)
		self.cmd_port = cmd_port
		self.obs_port = obs_port
		self.cameras = cameras
		self.received: list[dict[str, float]] = []
		self.stop_event = threading.Event()

	def run(self) -> None:
		context = zmq.Context()
		cmd = context.socket(zmq.PULL)
		cmd.setsockopt(zmq.CONFLATE, 1)
		cmd.bind(f'tcp://*:{self.cmd_port}')
		obs = context.socket(zmq.PUSH)
		obs.setsockopt(zmq.CONFLATE, 1)
		obs.bind(f'tcp://*:{self.obs_port}')
		state = {
			f'{prefix}_{joint}.pos': float(index)
			for prefix in ('left_arm', 'right_arm')
			for index, joint in enumerate(JOINTS)
		}
		sequence = 0
		while not self.stop_event.is_set():
			try:
				self.received.append(json.loads(cmd.recv_string(zmq.NOBLOCK)))
			except zmq.Again:
				pass
			sequence += 1
			state['left_arm_shoulder_pan.pos'] = float(sequence)
			frames = {
				name: np.full((height, width, 3), 90, dtype=np.uint8)
				for name, (height, width) in self.cameras.items()
			}
			try:
				obs.send_string(
					encode_observation(
						{**state, **frames}, tuple(frames),
						timestamp=time.time(), sequence=sequence,
					),
					zmq.NOBLOCK,
				)
			except zmq.Again:
				pass
			time.sleep(1.0 / HOST_HZ)
		cmd.close(linger=0)
		obs.close(linger=0)
		context.term()

	def stop(self) -> None:
		self.stop_event.set()
		self.join(timeout=2.0)


def _make_config(
	port_base: int, cameras: dict[str, tuple[int, int]], **overrides: float,
) -> BiSo101ClientConfig:
	return BiSo101ClientConfig(
		remote_ip='127.0.0.1',
		port_zmq_cmd=port_base,
		port_zmq_observations=port_base + 1,
		cameras={
			name: OpenCVCameraConfig(
				index_or_path=0, width=width, height=height, fps=30,
			)
			for name, (height, width) in cameras.items()
		},
		id='test_client',
		**overrides,
	)


def test_roundtrip_fresh_after_pause_and_nan_filtered() -> None:
	host = _FakeHost(BASE_PORT, BASE_PORT + 1, {'top': (480, 640)})
	host.start()
	time.sleep(0.2)
	robot = BiSo101Client(_make_config(BASE_PORT, {'top': (480, 640)}))
	robot.connect()
	try:
		first = robot.get_observation()
		assert set(first) == set(robot.observation_features)
		assert first['top'].shape == (480, 640, 3)
		# 제어 루프가 1초 멈춰도(에피소드 저장 상황) 최신 관측이 와야 한다.
		time.sleep(1.0)
		later = robot.get_observation()
		assert (
			later['left_arm_shoulder_pan.pos']
			- first['left_arm_shoulder_pan.pos'] > 20
		)
		sent = robot.send_action({
			'left_arm_gripper.pos': 42.0,
			'right_arm_gripper.pos': float('nan'),
			'ignored': 1.0,
		})
		time.sleep(0.3)
		assert sent == {'left_arm_gripper.pos': 42.0}
		assert host.received[-1] == {'left_arm_gripper.pos': 42.0}
	finally:
		robot.disconnect()
		host.stop()


def test_stale_detection_when_host_stops() -> None:
	host = _FakeHost(BASE_PORT + 10, BASE_PORT + 11, {'top': (48, 64)})
	host.start()
	time.sleep(0.2)
	robot = BiSo101Client(
		_make_config(BASE_PORT + 10, {'top': (48, 64)}, stale_timeout_s=0.5)
	)
	robot.connect()
	try:
		robot.get_observation()
		host.stop()
		time.sleep(0.8)
		for call in (robot.get_observation, lambda: robot.send_action({})):
			try:
				call()
			except DeviceNotConnectedError:
				continue
			raise AssertionError('stale observation must raise')
	finally:
		robot.disconnect()


def test_connect_rejects_shape_mismatch_and_missing_camera() -> None:
	host = _FakeHost(BASE_PORT + 20, BASE_PORT + 21, {'top': (480, 640)})
	host.start()
	time.sleep(0.2)
	try:
		for cameras in (
			{'top': (240, 320)},  # shape 불일치
			{'top': (480, 640), 'wrist': (480, 640)},  # host에 없는 카메라
		):
			robot = BiSo101Client(
				_make_config(BASE_PORT + 20, cameras, connect_timeout_s=1.0)
			)
			try:
				robot.connect()
			except DeviceNotConnectedError:
				assert not robot.is_connected
				continue
			raise AssertionError(f'connect must fail for {cameras}')
	finally:
		host.stop()


def test_connect_times_out_without_host() -> None:
	robot = BiSo101Client(
		_make_config(BASE_PORT + 30, {'top': (48, 64)}, connect_timeout_s=0.5)
	)
	started = time.monotonic()
	try:
		robot.connect()
	except DeviceNotConnectedError:
		assert time.monotonic() - started < 3.0
	else:
		raise AssertionError('connect must time out without a host')
	assert not any(
		'client_rx' in thread.name for thread in threading.enumerate()
	), 'receiver thread must not survive a failed connect'


if __name__ == '__main__':
	for name, test in list(globals().items()):
		if name.startswith('test_') and callable(test):
			test()
			print(f'ok  {name}')
