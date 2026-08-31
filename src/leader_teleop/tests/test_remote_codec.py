"""
remote_codec 직렬화 단위 테스트
===============================
하드웨어·네트워크 없이 관측 인코딩/디코딩 왕복을 검증한다.

실행:
	PYTHONPATH=src python -m leader_teleop.tests.test_remote_codec
	(pytest 호환: PYTHONPATH=src pytest src/leader_teleop/tests)
"""

import numpy as np

from leader_teleop.remote_codec import (
	decode_observation,
	encode_observation,
)


def _make_frame(height: int, width: int, value: int) -> np.ndarray:
	"""단색 테스트 프레임을 만든다 (JPEG 손실에도 값이 유지되게 단색)."""
	return np.full((height, width, 3), value, dtype=np.uint8)


def test_state_roundtrip_exact() -> None:
	observation = {
		'left_arm_shoulder_pan.pos': 12.5,
		'right_arm_gripper.pos': np.float32(3.25),
	}
	state, frames = decode_observation(
		encode_observation(observation, camera_keys=()), camera_keys=()
	)
	assert state == {
		'left_arm_shoulder_pan.pos': 12.5,
		'right_arm_gripper.pos': 3.25,
	}
	assert frames == {}


def test_frame_roundtrip_keeps_shape_and_approx_values() -> None:
	frame = _make_frame(48, 64, 200)
	observation = {'top': frame, 'left_arm_gripper.pos': 1.0}
	state, frames = decode_observation(
		encode_observation(observation, camera_keys=('top',)),
		camera_keys=('top',),
	)
	assert state == {'left_arm_gripper.pos': 1.0}
	assert frames['top'].shape == (48, 64, 3)
	assert frames['top'].dtype == np.uint8
	assert abs(int(frames['top'].mean()) - 200) <= 2  # JPEG 손실 허용


def test_undeclared_camera_is_ignored_not_fatal() -> None:
	observation = {
		'top': _make_frame(8, 8, 10),
		'wrist': _make_frame(8, 8, 20),
		'left_arm_gripper.pos': 4.0,
	}
	message = encode_observation(observation, camera_keys=('top', 'wrist'))
	# client가 'wrist'를 선언하지 않은 경우: 프레임 문자열은 버리고
	# 상태와 선언된 카메라만 살아남아야 한다 (루프가 죽으면 안 된다).
	state, frames = decode_observation(message, camera_keys=('top',))
	assert state == {'left_arm_gripper.pos': 4.0}
	assert set(frames) == {'top'}


def test_empty_frame_string_is_skipped() -> None:
	message = '{"top": "", "left_arm_gripper.pos": 0.5}'
	state, frames = decode_observation(message, camera_keys=('top',))
	assert state == {'left_arm_gripper.pos': 0.5}
	assert frames == {}


if __name__ == '__main__':
	for name, test in list(globals().items()):
		if name.startswith('test_') and callable(test):
			test()
			print(f'ok  {name}')
