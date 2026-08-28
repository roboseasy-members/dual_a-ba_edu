"""실기 → 시뮬: SO-101 디지털 트윈.

팔로워암의 관절값을 읽어 MuJoCo 뷰어의 SO-101 모델에 30Hz로
미러링합니다. 연결 직후 토크를 해제하므로 손으로 실기 팔을 움직이면
화면 속 로봇이 따라옵니다.

⚠ 실행 전에 팔을 손으로 잡거나 눕혀 두세요 — 토크가 풀리면 팔이
중력에 처집니다.

Example:
	python -m src.mujoco_sim.digital_twin \\
		--port /dev/ttyACM0 --id my_follower
"""

import argparse
import time
from pathlib import Path
from typing import Final

import mujoco
import mujoco.viewer

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from .sim_bridge import obs_to_qpos

LOOP_HZ: Final[float] = 30.0
DEFAULT_SCENE: Final[Path] = (
	Path(__file__).resolve().parent / 'so101' / 'scene.xml')


def parse_args() -> argparse.Namespace:
	"""CLI 인자를 파싱한다."""
	parser = argparse.ArgumentParser(
		description='실기 SO-101 관절값을 MuJoCo 뷰어에 미러링한다.')
	parser.add_argument(
		'--port', required=True,
		help='팔로워암 시리얼 포트 (예: /dev/ttyACM0)')
	parser.add_argument(
		'--id', required=True, help='캘리브레이션에 쓴 로봇 id')
	parser.add_argument(
		'--scene', default=str(DEFAULT_SCENE),
		help='SO-101 scene.xml 경로 (기본: 레포 동봉 모델)')
	return parser.parse_args()


def run_twin(port: str, robot_id: str, scene_path: Path) -> None:
	"""실기 관절값을 읽어 MuJoCo 모델에 미러링한다.

	Args:
		port: 팔로워암 시리얼 포트.
		robot_id: 캘리브레이션 파일과 연결된 로봇 id.
		scene_path: SO-101 MJCF scene 파일 경로.
	"""
	model = mujoco.MjModel.from_xml_path(str(scene_path))
	data = mujoco.MjData(model)

	robot = SO101Follower(SO101FollowerConfig(port=port, id=robot_id))
	robot.connect()
	robot.bus.disable_torque()  # 손으로 움직일 수 있게 토크 해제

	try:
		with mujoco.viewer.launch_passive(model, data) as viewer:
			while viewer.is_running():
				obs_to_qpos(model, data, robot.get_observation())
				mujoco.mj_forward(model, data)  # 물리 적분 없이 자세만 갱신
				viewer.sync()
				time.sleep(1.0 / LOOP_HZ)
	finally:
		robot.disconnect()


def main() -> None:
	"""엔트리 포인트."""
	args = parse_args()
	run_twin(args.port, args.id, Path(args.scene).expanduser())


if __name__ == '__main__':
	main()
