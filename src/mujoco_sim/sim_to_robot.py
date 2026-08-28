"""시뮬 → 실기: MuJoCo 뷰어로 SO-101 구동.

MuJoCo 뷰어의 Control 슬라이더로 시뮬 로봇을 움직이면, 시뮬 관절값이
실기 팔로워암으로 전송됩니다. 시작 시 시뮬 자세를 실기 현재 자세에
맞추므로 실행 직후 팔이 튀지 않습니다.

⚠ 실기가 움직이는 데모입니다.
- 시연 전 팔 주변 공간을 확보하세요.
- 스텝당 이동량은 --max_step_deg(기본 3도)로 잘립니다. 전송 주기
  15Hz 기준 최대 약 45도/초입니다.
- 비상 시 뷰어 창을 닫거나 Ctrl+C — 즉시 전송이 멈춥니다. 그래도
  위험하면 전원을 차단하세요.

Example:
	python -m src.mujoco_sim.sim_to_robot \\
		--port /dev/ttyACM0 --id my_follower
"""

import argparse
import time
from pathlib import Path
from typing import Final

import mujoco
import mujoco.viewer

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from .sim_bridge import ALL_JOINTS, get_qpos_index, obs_to_qpos, \
	qpos_to_action

SEND_HZ: Final[float] = 15.0
DEFAULT_MAX_STEP_DEG: Final[float] = 3.0
DEFAULT_SCENE: Final[Path] = (
	Path(__file__).resolve().parent / 'so101' / 'scene.xml')


def parse_args() -> argparse.Namespace:
	"""CLI 인자를 파싱한다."""
	parser = argparse.ArgumentParser(
		description='MuJoCo 뷰어 슬라이더로 실기 SO-101을 구동한다.')
	parser.add_argument(
		'--port', required=True,
		help='팔로워암 시리얼 포트 (예: /dev/ttyACM0)')
	parser.add_argument(
		'--id', required=True, help='캘리브레이션에 쓴 로봇 id')
	parser.add_argument(
		'--scene', default=str(DEFAULT_SCENE),
		help='SO-101 scene.xml 경로 (기본: 레포 동봉 모델)')
	parser.add_argument(
		'--max_step_deg', type=float, default=DEFAULT_MAX_STEP_DEG,
		help='전송 1회당 관절 이동량 상한 (deg)')
	return parser.parse_args()


def sync_sim_to_robot_pose(
	model: mujoco.MjModel,
	data: mujoco.MjData,
	robot: SO101Follower,
) -> None:
	"""시뮬 자세(qpos·ctrl)를 실기 현재 자세로 맞춘다.

	실행 직후 실기가 시뮬의 기본 자세로 튀는 것을 막는다.
	"""
	obs_to_qpos(model, data, robot.get_observation())
	for joint_name in ALL_JOINTS:
		actuator_id = mujoco.mj_name2id(
			model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
		data.ctrl[actuator_id] = data.qpos[get_qpos_index(model, joint_name)]
	mujoco.mj_forward(model, data)


def run_sim_to_robot(
	port: str,
	robot_id: str,
	scene_path: Path,
	max_step_deg: float,
) -> None:
	"""뷰어 슬라이더 조작을 실기 팔로워암으로 전송한다.

	Args:
		port: 팔로워암 시리얼 포트.
		robot_id: 캘리브레이션 파일과 연결된 로봇 id.
		scene_path: SO-101 MJCF scene 파일 경로.
		max_step_deg: 전송 1회당 관절 이동량 상한 (deg).
	"""
	model = mujoco.MjModel.from_xml_path(str(scene_path))
	data = mujoco.MjData(model)

	robot = SO101Follower(SO101FollowerConfig(
		port=port, id=robot_id, max_relative_target=max_step_deg))
	robot.connect()

	steps_per_send = max(1, int(1.0 / SEND_HZ / model.opt.timestep))
	try:
		sync_sim_to_robot_pose(model, data, robot)
		with mujoco.viewer.launch_passive(model, data) as viewer:
			while viewer.is_running():
				for _ in range(steps_per_send):  # 슬라이더(ctrl) → 시뮬 관절
					mujoco.mj_step(model, data)
				robot.send_action(qpos_to_action(model, data))
				viewer.sync()
				time.sleep(1.0 / SEND_HZ)
	finally:
		robot.disconnect()


def main() -> None:
	"""엔트리 포인트."""
	args = parse_args()
	run_sim_to_robot(
		args.port, args.id, Path(args.scene).expanduser(),
		args.max_step_deg)


if __name__ == '__main__':
	main()
