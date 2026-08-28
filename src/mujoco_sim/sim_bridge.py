"""실기(lerobot) ↔ 시뮬(MuJoCo) 관절값 변환 브리지.

단위 규칙을 한 곳에 모아둔 모듈입니다. digital_twin.py(실기→시뮬)와
sim_to_robot.py(시뮬→실기)가 공유합니다.

- lerobot SO-101 관측/액션: 팔 5관절은 도(deg), 그리퍼는 0~100 정규화
- MuJoCo 관절(qpos): 라디안(rad), 그리퍼는 모델의 jnt_range 안의 각도

관절 이름 6개는 SO-ARM100 저장소의 MJCF와 lerobot 모터 이름이
동일합니다 — 그래서 이름 매핑 테이블 없이 그대로 잇습니다.
"""

import math
from typing import Final

import mujoco

ARM_JOINTS: Final[tuple[str, ...]] = (
	'shoulder_pan',
	'shoulder_lift',
	'elbow_flex',
	'wrist_flex',
	'wrist_roll',
)
GRIPPER_JOINT: Final[str] = 'gripper'
ALL_JOINTS: Final[tuple[str, ...]] = ARM_JOINTS + (GRIPPER_JOINT,)


def get_qpos_index(model: mujoco.MjModel, joint_name: str) -> int:
	"""관절 이름으로 qpos 배열 인덱스를 찾는다.

	Args:
		model: MuJoCo 모델.
		joint_name: MJCF 관절 이름.

	Returns:
		data.qpos에서 이 관절이 차지하는 인덱스.
	"""
	joint_id = mujoco.mj_name2id(
		model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
	return int(model.jnt_qposadr[joint_id])


def get_gripper_range(model: mujoco.MjModel) -> tuple[float, float]:
	"""그리퍼 관절의 가동범위(rad)를 돌려준다."""
	joint_id = mujoco.mj_name2id(
		model, mujoco.mjtObj.mjOBJ_JOINT, GRIPPER_JOINT)
	range_lo, range_hi = model.jnt_range[joint_id]
	return float(range_lo), float(range_hi)


def obs_to_qpos(
	model: mujoco.MjModel,
	data: mujoco.MjData,
	obs: dict[str, float],
) -> None:
	"""실기 관측('{모터}.pos')을 시뮬 qpos에 써 넣는다.

	Args:
		model: MuJoCo 모델.
		data: MuJoCo 상태 (qpos가 갱신된다).
		obs: robot.get_observation() 결과. 팔은 deg, 그리퍼는 0~100.
	"""
	for joint_name in ARM_JOINTS:
		qpos_index = get_qpos_index(model, joint_name)
		data.qpos[qpos_index] = math.radians(obs[f'{joint_name}.pos'])
	range_lo, range_hi = get_gripper_range(model)
	gripper_ratio = obs[f'{GRIPPER_JOINT}.pos'] / 100.0
	qpos_index = get_qpos_index(model, GRIPPER_JOINT)
	data.qpos[qpos_index] = (
		range_lo + (range_hi - range_lo) * gripper_ratio)


def qpos_to_action(
	model: mujoco.MjModel,
	data: mujoco.MjData,
) -> dict[str, float]:
	"""시뮬 qpos를 실기 액션('{모터}.pos')으로 변환한다.

	Args:
		model: MuJoCo 모델.
		data: MuJoCo 상태.

	Returns:
		robot.send_action()에 넣을 dict. 팔은 deg, 그리퍼는 0~100.
	"""
	action: dict[str, float] = {}
	for joint_name in ARM_JOINTS:
		qpos_index = get_qpos_index(model, joint_name)
		action[f'{joint_name}.pos'] = math.degrees(data.qpos[qpos_index])
	range_lo, range_hi = get_gripper_range(model)
	qpos_index = get_qpos_index(model, GRIPPER_JOINT)
	gripper_ratio = (
		(data.qpos[qpos_index] - range_lo) / (range_hi - range_lo))
	action[f'{GRIPPER_JOINT}.pos'] = min(max(gripper_ratio, 0.0), 1.0) * 100.0
	return action
