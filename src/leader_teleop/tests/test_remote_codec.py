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
	SEQUENCE_KEY,
	TIMESTAMP_KEY,
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
	decoded = decode_observation(
		encode_observation(observation, camera_keys=()), camera_keys=()
	)
	assert decoded.state == {
		'left_arm_shoulder_pan.pos': 12.5,
		'right_arm_gripper.pos': 3.25,
	}
	assert decoded.frames == {}
	assert decoded.timestamp is None and decoded.sequence is None


def test_frame_roundtrip_keeps_shape_and_approx_values() -> None:
	frame = _make_frame(48, 64, 200)
	observation = {'top': frame, 'left_arm_gripper.pos': 1.0}
	decoded = decode_observation(
		encode_observation(observation, camera_keys=('top',)),
		camera_keys=('top',),
	)
	assert decoded.state == {'left_arm_gripper.pos': 1.0}
	assert decoded.frames['top'].shape == (48, 64, 3)
	assert decoded.frames['top'].dtype == np.uint8
	assert abs(int(decoded.frames['top'].mean()) - 200) <= 2  # JPEG 손실


def test_channel_order_is_preserved() -> None:
	# R/G/B가 서로 다른 값이어야 채널 스왑을 잡는다.
	frame = np.zeros((32, 32, 3), dtype=np.uint8)
	frame[..., 0], frame[..., 1], frame[..., 2] = 200, 100, 30
	decoded = decode_observation(
		encode_observation({'top': frame}, camera_keys=('top',)),
		camera_keys=('top',),
	)
	means = decoded.frames['top'].reshape(-1, 3).mean(axis=0)
	assert abs(means[0] - 200) <= 4 and abs(means[1] - 100) <= 4
	assert abs(means[2] - 30) <= 4


def test_meta_roundtrip_and_excluded_from_state() -> None:
	message = encode_observation(
		{'left_arm_gripper.pos': 2.0}, camera_keys=(),
		timestamp=1234.5, sequence=7,
	)
	decoded = decode_observation(message, camera_keys=())
	assert decoded.timestamp == 1234.5 and decoded.sequence == 7
	assert TIMESTAMP_KEY not in decoded.state
	assert SEQUENCE_KEY not in decoded.state
	assert decoded.state == {'left_arm_gripper.pos': 2.0}


def test_undeclared_camera_is_ignored_not_fatal() -> None:
	observation = {
		'top': _make_frame(8, 8, 10),
		'wrist': _make_frame(8, 8, 20),
		'left_arm_gripper.pos': 4.0,
	}
	message = encode_observation(observation, camera_keys=('top', 'wrist'))
	# client가 'wrist'를 선언하지 않은 경우: 프레임 문자열은 버리고
	# 상태와 선언된 카메라만 살아남아야 한다 (루프가 죽으면 안 된다).
	decoded = decode_observation(message, camera_keys=('top',))
	assert decoded.state == {'left_arm_gripper.pos': 4.0}
	assert set(decoded.frames) == {'top'}


def test_empty_frame_string_is_skipped() -> None:
	message = '{"top": "", "left_arm_gripper.pos": 0.5}'
	decoded = decode_observation(message, camera_keys=('top',))
	assert decoded.state == {'left_arm_gripper.pos': 0.5}
	assert decoded.frames == {}


def test_non_object_payload_raises_value_error() -> None:
	for bad_message in ('[1, 2, 3]', '"text"', 'not json'):
		try:
			decode_observation(bad_message, camera_keys=())
		except ValueError:
			continue
		raise AssertionError(f'ValueError expected for {bad_message!r}')


if __name__ == '__main__':
	for name, test in list(globals().items()):
		if name.startswith('test_') and callable(test):
			test()
			print(f'ok  {name}')
