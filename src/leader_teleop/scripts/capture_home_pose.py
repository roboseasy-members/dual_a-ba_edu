"""
홈포즈(수그린 자세) 캡처 유틸리티
=================================
로봇을 연결한 뒤 토크를 해제하고, 사용자가 양팔/머리를 원하는 홈포즈로
직접 잡으면 현재 관절각을 읽어 HomeReturnConfig(config.py)에 그대로
붙여넣을 수 있는 형식으로 출력합니다. 팔 값은 좌/우 평균을 제안하되
좌/우 원본 값도 함께 보여줍니다.

실행:
	PYTHONPATH=src python -m leader_teleop.scripts.capture_home_pose \
		--left-arm-port /dev/ttyACM0 --right-arm-port /dev/ttyACM1 \
		--robot-id bi_so101_follower
	# SO-102: --robot-type bi_so102_follower
	# 머리 미장착: --no-head
"""

import argparse

from ..config import GRIPPER_NAME
from ..head.head_modes import PAN_MOTOR_KEY, TILT_MOTOR_KEY
from ..robots import (
	BiFollowerBase,
	BiSo101Follower,
	BiSo102Follower,
)

# --robot-type 문자열 -> 로봇 클래스.
ROBOT_CLASSES: dict[str, type[BiFollowerBase]] = {
	'bi_so101_follower': BiSo101Follower,
	'bi_so102_follower': BiSo102Follower,
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
	"""커맨드라인 인자를 파싱한다.

	Args:
		argv: 인자 목록 (None이면 sys.argv 사용).

	Returns:
		파싱된 네임스페이스.
	"""
	parser = argparse.ArgumentParser(
		description='홈포즈(수그린 자세) 관절각 캡처 - HomeReturnConfig용'
	)
	parser.add_argument(
		'--robot-type',
		choices=sorted(ROBOT_CLASSES),
		default='bi_so101_follower',
		help='로봇 세대 (teleoperate/record의 --robot.type과 동일).',
	)
	parser.add_argument(
		'--left-arm-port', default='/dev/ttyACM0', help='bus1 포트 (왼팔+머리).',
	)
	parser.add_argument(
		'--right-arm-port', default='/dev/ttyACM1', help='bus2 포트 (오른팔).',
	)
	parser.add_argument(
		'--robot-id',
		default='bi_so101_follower',
		help='캘리브레이션 파일 식별용 로봇 id.',
	)
	parser.add_argument(
		'--no-head',
		action='store_true',
		help='머리 모터 미장착 구성 (bus1이 팔 모터만 검색).',
	)
	return parser.parse_args(argv)


def print_captured_pose(
	robot: BiFollowerBase, observation: dict[str, float],
) -> None:
	"""관측값을 좌/우 비교표와 config 붙여넣기 블록으로 출력한다.

	Args:
		robot: 관절 명세를 제공하는 로봇 (arm_joint_specs).
		observation: 로봇 관측 딕셔너리 (팔 관절 .pos 키 포함).
	"""
	joint_names = [
		joint for joint, _ in robot.arm_joint_specs
	] + [GRIPPER_NAME]

	print('\nCurrent joint angles (deg, gripper 0~100):')
	print(f'{"joint":<16}{"left":>10}{"right":>10}{"avg":>10}')
	averages: dict[str, float] = {}
	for name in joint_names:
		left_value = float(observation[f'left_arm_{name}.pos'])
		right_value = float(observation[f'right_arm_{name}.pos'])
		averages[name] = (left_value + right_value) / 2.0
		print(
			f'{name:<16}{left_value:>10.1f}{right_value:>10.1f}'
			f'{averages[name]:>10.1f}'
		)

	has_head = PAN_MOTOR_KEY in observation
	if has_head:
		head_pan = float(observation[PAN_MOTOR_KEY])
		head_tilt = float(observation[TILT_MOTOR_KEY])
		print(f'{"head_pan":<16}{head_pan:>30.1f}')
		print(f'{"head_tilt":<16}{head_tilt:>30.1f}')

	print(
		'\nValues to paste into HomeReturnConfig in config.py '
		'(arms averaged):\n'
	)
	for joint, _ in robot.arm_joint_specs:
		print(f'\t{joint}_deg: float = {averages[joint]:.1f}')
	print(f'\tgripper_pos: float = {averages[GRIPPER_NAME]:.1f}')
	if has_head:
		print(f'\thead_pan_deg: float = {head_pan:.1f}')
		print(f'\thead_tilt_deg: float = {head_tilt:.1f}')
	print(
		'\nIf left/right differ a lot, pick the side that fits your '
		'setup instead of the average, or re-pose and capture again.'
	)


def main(argv: list[str] | None = None) -> None:
	"""로봇 연결 -> 토크 해제 -> 수동 자세 -> 관절각 출력.

	Args:
		argv: 인자 목록 (None이면 sys.argv 사용).
	"""
	args = parse_args(argv)
	robot_class = ROBOT_CLASSES[args.robot_type]
	robot = robot_class(
		robot_class.config_class(
			id=args.robot_id,
			left_arm_port=args.left_arm_port,
			right_arm_port=args.right_arm_port,
			has_head_motors=not args.no_head,
		)
	)
	try:
		robot.connect()
		robot.bus1.disable_torque()
		robot.bus2.disable_torque()
		input(
			'\nTorque disabled. Manually pose both arms (and the head) '
			'into the desired home (rest) pose, then press ENTER...'
		)
		print_captured_pose(robot, robot.get_observation())
	except KeyboardInterrupt:
		print('\nCapture aborted (Ctrl+C).')
	finally:
		if robot.is_connected:
			robot.disconnect()


if __name__ == '__main__':
	main()
