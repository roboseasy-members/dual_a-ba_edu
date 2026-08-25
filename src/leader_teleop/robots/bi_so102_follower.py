"""
SO-102 양팔 + 머리 팔로워
=========================
공통 구현은 bi_follower_base 참조. SO-102는 SO-101 대비 손목에 yaw 축이
추가된 7모터 팔입니다 (4번까지 SO-101과 스펙 동일):

	1 shoulder_pan / 2 shoulder_lift / 3 elbow_flex /
	4 wrist_flex(pitch) / 5 wrist_yaw(신규) / 6 wrist_roll / 7 gripper

카메라 헤드 모터 id는 8/9 (SO-101은 7/8).
"""

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig

from .bi_follower_base import BiFollowerBase, BiFollowerBaseConfig


@RobotConfig.register_subclass('bi_so102_follower')
@dataclass
class BiSo102FollowerConfig(BiFollowerBaseConfig):
	"""SO-102 양팔+머리 로봇 설정 (공통 필드는 BiFollowerBaseConfig)."""


class BiSo102Follower(BiFollowerBase):
	"""SO-102 양팔(각 7모터, id 1~7) + 머리(id 8/9) 로봇."""

	config_class = BiSo102FollowerConfig
	name = 'bi_so102_follower'

	# 손목: flex(pitch) -> yaw -> roll. 관절명은 HomeReturnConfig의
	# '{관절명}_deg' 필드와 짝이다.
	arm_joint_specs = (
		('shoulder_pan', 1),
		('shoulder_lift', 2),
		('elbow_flex', 3),
		('wrist_flex', 4),
		('wrist_yaw', 5),
		('wrist_roll', 6),
	)
	gripper_motor_id = 7
	head_motor_ids = (8, 9)
