"""
로봇 버스 하드웨어 점검 스크립트 (SO-101/SO-102)
================================================
텔레옵 실행 전에 Feetech 버스를 스캔해 실물 모터 구성을 판정하고 알맞은
실행 플래그를 안내합니다. broadcast ping + 전압 읽기만 수행하는
**읽기 전용** 점검이라 모터가 움직이거나 토크가 바뀌지 않습니다.

두 모드:
	1. 포트 미지정(기본) - /dev/ttyACM*를 전부 스캔해 각 포트를
	   팔로워 bus1(머리 포함)/팔로워 팔/리더암(전압 5V)으로 분류하고
	   포트 배치를 제안한다. 포트가 재열거로 뒤바뀌었을 때 유용.
	2. --left-arm-port/--right-arm-port 지정 - 해당 두 포트가 선택한
	   --robot-type 구성과 일치하는지 판정한다.

기대 모터 배치는 robots/의 로봇 클래스 명세(arm_joint_specs 등)에서
파생하므로 여기에 id를 중복 하드코딩하지 않습니다.

실행 (레포 루트에서):
	PYTHONPATH=src python -m leader_teleop.scripts.check_robot
	PYTHONPATH=src python -m leader_teleop.scripts.check_robot \\
		--robot-type bi_so101_follower \\
		--left-arm-port /dev/ttyACM0 --right-arm-port /dev/ttyACM1
"""

import argparse
import glob
from typing import Final

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from leader_teleop.robots.bi_so101_follower import BiSo101Follower
from leader_teleop.robots.bi_so102_follower import BiSo102Follower


# sts3215 모델 번호 (broadcast ping 응답 검증용).
STS3215_MODEL_NUMBER: Final[int] = 777
# 팔로워(12V) / 리더(5V) 전원 판별 임계값.
FOLLOWER_VOLTAGE_MIN_V: Final[float] = 10.8
FOLLOWER_VOLTAGE_MAX_V: Final[float] = 13.2
LEADER_VOLTAGE_MAX_V: Final[float] = 8.0
PING_NUM_RETRY: Final[int] = 3

ROBOT_CLASSES: Final[dict] = {
	'bi_so101_follower': BiSo101Follower,
	'bi_so102_follower': BiSo102Follower,
}


def get_arm_ids(robot_class) -> tuple[int, ...]:
	"""로봇 클래스 명세에서 팔(그리퍼 포함) 모터 id를 뽑는다."""
	ids = [motor_id for _joint, motor_id in robot_class.arm_joint_specs]
	ids.append(robot_class.gripper_motor_id)
	return tuple(sorted(ids))


def get_id_names(robot_class, has_head: bool) -> dict[int, str]:
	"""로봇 클래스 명세에서 모터 id -> 관절명 매핑을 만든다."""
	id_names = {
		motor_id: joint for joint, motor_id in robot_class.arm_joint_specs
	}
	id_names[robot_class.gripper_motor_id] = 'gripper'
	if has_head:
		pan_id, tilt_id = robot_class.head_motor_ids
		id_names[pan_id] = 'head_motor_1 (pan)'
		id_names[tilt_id] = 'head_motor_2 (tilt)'
	return id_names


def scan_bus(port: str, label: str) -> dict[int, int] | None:
	"""포트를 열어 broadcast ping으로 (id -> 모델번호)를 수집한다.

	Args:
		port: 시리얼 포트 경로.
		label: 로그에 붙일 이름.

	Returns:
		발견한 {id: model_number} (없으면 빈 dict). 실패 시 None.
	"""
	try:
		bus = FeetechMotorsBus(port, {})
		bus._connect(handshake=False)
	except Exception as error:
		print(f'  {label}: cannot open port ({error})')
		print('  -> Check the cable, port name, and permissions '
			'(dialout group).')
		return None
	try:
		found = bus.broadcast_ping(num_retry=PING_NUM_RETRY)
	except Exception as error:
		print(f'  {label}: scan failed ({type(error).__name__}: {error})')
		print('  -> Not a responding motor bus, a re-enumerated port, '
			'or another process is using it.')
		return None
	finally:
		bus.port_handler.closePort()
	return dict(sorted(found.items())) if found else {}


def read_voltages(
	port: str, motor_ids: list[int],
) -> dict[int, float | None]:
	"""모터들의 입력 전압을 읽는다 (best-effort, 실패한 id는 None).

	Args:
		port: 시리얼 포트 경로.
		motor_ids: 읽을 모터 id 목록.

	Returns:
		{id: 전압(V) 또는 None}.
	"""
	motors = {
		f'id_{motor_id}': Motor(
			motor_id, 'sts3215', MotorNormMode.DEGREES
		)
		for motor_id in motor_ids
	}
	voltages: dict[int, float | None] = dict.fromkeys(motor_ids)
	try:
		bus = FeetechMotorsBus(port, motors)
		bus._connect(handshake=False)
	except Exception:
		return voltages
	try:
		for motor_id in motor_ids:
			try:
				raw_voltage = bus.read(
					'Present_Voltage', f'id_{motor_id}',
					normalize=False, num_retry=PING_NUM_RETRY,
				)
				voltages[motor_id] = raw_voltage / 10
			except Exception:
				pass
	finally:
		bus.port_handler.closePort()
	return voltages


def classify_layout(found_ids: set[int]) -> tuple[str, str] | None:
	"""발견 id 집합이 어느 (robot_type, 구성)인지 판정한다.

	Args:
		found_ids: 스캔으로 발견한 모터 id 집합.

	Returns:
		(robot_type, 'arm'|'arm+head') 또는 None(불일치).
	"""
	for robot_type, robot_class in ROBOT_CLASSES.items():
		arm_ids = set(get_arm_ids(robot_class))
		if found_ids == arm_ids:
			return robot_type, 'arm'
		if found_ids == arm_ids | set(robot_class.head_motor_ids):
			return robot_type, 'arm+head'
	return None


def classify_power(voltage: float | None) -> str:
	"""대표 전압으로 팔로워(12V)/리더(5V) 전원을 분류한다."""
	if voltage is None:
		return 'unknown'
	if voltage >= FOLLOWER_VOLTAGE_MIN_V:
		return 'follower-12V'
	if voltage <= LEADER_VOLTAGE_MAX_V:
		return 'leader-5V'
	return f'unusual ({voltage:.1f}V)'


def run_discovery() -> None:
	"""모든 ACM 포트를 스캔·분류하고 포트 배치를 제안한다."""
	ports = sorted(glob.glob('/dev/ttyACM*'))
	if not ports:
		raise SystemExit('FAIL: no /dev/ttyACM* ports found.')
	print(f'[discover] scanning {len(ports)} ports: {ports}')

	results = []
	for port in ports:
		found = scan_bus(port, port)
		if found is None or not found:
			results.append((port, None, None, None))
			continue
		layout = classify_layout(set(found))
		voltage = read_voltages(port, [min(found)])[min(found)]
		results.append((port, found, layout, voltage))
		layout_text = (
			f'{layout[0]} {layout[1]}' if layout
			else f'unknown layout {sorted(found)}'
		)
		print(f'  {port}: ids {sorted(found)} -> {layout_text}, '
			f'power {classify_power(voltage)}')

	bus1_candidates = []
	follower_arm_candidates = []
	leader_candidates = []
	for port, found, layout, voltage in results:
		if not found or layout is None:
			continue
		power = classify_power(voltage)
		if layout[1] == 'arm+head':
			bus1_candidates.append((port, layout[0]))
		elif power == 'follower-12V':
			follower_arm_candidates.append((port, layout[0]))
		elif power == 'leader-5V':
			leader_candidates.append((port, layout[0]))

	print('[suggestion]')
	if len(bus1_candidates) == 1 and len(follower_arm_candidates) == 1:
		bus1_port, robot_type = bus1_candidates[0]
		right_port, _right_type = follower_arm_candidates[0]
		print(f'  follower: --robot.type={robot_type} \\')
		print(f'    --robot.left_arm_port={bus1_port} '
			f'--robot.right_arm_port={right_port}')
		print('    (bus1 = the port with the head motors)')
	elif not bus1_candidates:
		print('  no port has head motors - if the head is not mounted, '
			'left/right follower arms cannot be told apart by scan. '
			'Identify them by unplugging one arm.')
	else:
		print('  ambiguous follower layout - candidates:')
		for port, robot_type in bus1_candidates:
			print(f'    bus1? {port} ({robot_type})')
		for port, robot_type in follower_arm_candidates:
			print(f'    right arm? {port} ({robot_type})')

	if leader_candidates:
		leader_ports = [port for port, _robot_type in leader_candidates]
		print(f'  leader arms (5V): {leader_ports} - left/right is not '
			'detectable by scan; verify by moving one arm '
			'(--teleop.left_arm_port / --teleop.right_arm_port)')
	print('  note: ACM numbering shuffles on replug - consider udev '
		'symlink rules for stable names.')


def run_verdict(
	robot_type: str, left_arm_port: str, right_arm_port: str,
) -> None:
	"""지정한 두 포트가 로봇 타입 구성과 일치하는지 판정한다."""
	robot_class = ROBOT_CLASSES[robot_type]
	arm_ids = get_arm_ids(robot_class)
	head_ids = tuple(robot_class.head_motor_ids)
	print(f'[expect] {robot_type}: arm ids {list(arm_ids)} per bus, '
		f'head ids {list(head_ids)} on bus1 (only if head mounted)')

	print('[scan] bus1 (left arm + head)')
	bus1_found = scan_bus(left_arm_port, 'bus1')
	print('[scan] bus2 (right arm)')
	bus2_found = scan_bus(right_arm_port, 'bus2')
	if bus1_found is None or bus2_found is None:
		raise SystemExit('\nFAIL: could not scan both ports.')

	for label, found in (('bus1', bus1_found), ('bus2', bus2_found)):
		print(f'[found] {label}: {sorted(found) if found else "(none)"}')
		for motor_id, model_number in found.items():
			if model_number != STS3215_MODEL_NUMBER:
				print(f'  {label} id {motor_id}: unexpected model '
					f'{model_number} (expected sts3215='
					f'{STS3215_MODEL_NUMBER})')

	print('[voltage] (follower expects ~12V)')
	id_names = get_id_names(robot_class, has_head=True)
	for label, port, found in (
		('bus1', left_arm_port, bus1_found),
		('bus2', right_arm_port, bus2_found),
	):
		voltages = read_voltages(port, sorted(found))
		for motor_id, voltage in voltages.items():
			joint = id_names.get(motor_id, '?')
			if voltage is None:
				print(f'  {label} id {motor_id:>2} [{joint}]: '
					'read failed')
				continue
			is_ok = (
				FOLLOWER_VOLTAGE_MIN_V
				<= voltage <= FOLLOWER_VOLTAGE_MAX_V
			)
			mark = '' if is_ok else '  <-- OUT OF RANGE (12V?)'
			print(f'  {label} id {motor_id:>2} [{joint}]: '
				f'{voltage:.1f}V{mark}')

	print('[verdict]')
	expected_arm = set(arm_ids)
	is_bus2_ok = set(bus2_found) == expected_arm
	has_head = set(bus1_found) == expected_arm | set(head_ids)
	is_bus1_ok = has_head or set(bus1_found) == expected_arm
	if is_bus1_ok and is_bus2_ok:
		head_note = (
			'head mounted: use --camera_head.mode=fixed|keyboard'
			if has_head
			else 'head not mounted: omit --camera_head.mode'
		)
		leader_type = robot_type.replace('follower', 'leader')
		print(f'  PASS: matches {robot_type} ({head_note})')
		print('  run example:')
		print('    PYTHONPATH=src python -m leader_teleop.teleoperate \\')
		print(f'      --robot.type={robot_type} \\')
		print(f'      --robot.left_arm_port={left_arm_port} '
			f'--robot.right_arm_port={right_arm_port} \\')
		print(f'      --teleop.type={leader_type} \\')
		print('      --teleop.left_arm_port=<leader L> '
			'--teleop.right_arm_port=<leader R>'
			+ (' \\\n      --camera_head.mode=keyboard' if has_head else ''))
	else:
		print(f'  FAIL: scan does not match {robot_type}')
		print(f'    bus1 missing {sorted(expected_arm - set(bus1_found))}'
			f' extra {sorted(set(bus1_found) - expected_arm - set(head_ids))}')
		print(f'    bus2 missing {sorted(expected_arm - set(bus2_found))}'
			f' extra {sorted(set(bus2_found) - expected_arm)}')
		other = classify_layout(set(bus1_found))
		if other is not None and other[0] != robot_type:
			print(f'    -> bus1 looks like {other[0]} ({other[1]}) - '
				f'try --robot-type {other[0]}')
		print('    -> Or run without ports for auto-discovery: '
			'PYTHONPATH=src python -m leader_teleop.scripts.check_robot')


def main() -> None:
	"""포트 자동 탐색 또는 두 포트 판정 모드로 버스를 점검한다."""
	parser = argparse.ArgumentParser(
		description='Scan Feetech motor buses and verify the robot '
			'layout (read-only, no motion). Without ports, scans all '
			'/dev/ttyACM* and classifies each one.'
	)
	parser.add_argument(
		'--robot-type', default='bi_so101_follower',
		choices=sorted(ROBOT_CLASSES),
		help='expected robot generation (default: bi_so101_follower)',
	)
	parser.add_argument(
		'--left-arm-port', default=None,
		help='bus1 port - left arm (+ head). Omit for auto-discovery',
	)
	parser.add_argument(
		'--right-arm-port', default=None,
		help='bus2 port - right arm. Omit for auto-discovery',
	)
	args = parser.parse_args()

	if args.left_arm_port is None and args.right_arm_port is None:
		run_discovery()
		return
	if args.left_arm_port is None or args.right_arm_port is None:
		raise SystemExit(
			'Pass both --left-arm-port and --right-arm-port, '
			'or neither (auto-discovery).'
		)
	run_verdict(args.robot_type, args.left_arm_port, args.right_arm_port)


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		print('\nInterrupted by user (Ctrl+C).')
