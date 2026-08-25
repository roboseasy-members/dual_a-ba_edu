"""
SO-101 양팔 + 머리 팔로워
=========================
공통 구현은 bi_follower_base 참조. 이 파일은 SO-101 하드웨어 명세
(관절 5 + 그리퍼, 머리 id 7/8)만 선언합니다.
"""

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig

from .bi_follower_base import BiFollowerBase, BiFollowerBaseConfig


@RobotConfig.register_subclass('bi_so101_follower')
@dataclass
class BiSo101FollowerConfig(BiFollowerBaseConfig):
	"""SO-101 양팔+머리 로봇 설정 (공통 필드는 BiFollowerBaseConfig)."""


class BiSo101Follower(BiFollowerBase):
	"""SO-101 양팔(각 6모터, id 1~6) + 머리(id 7/8) 로봇."""

	config_class = BiSo101FollowerConfig
	name = 'bi_so101_follower'

	# 손목: flex(pitch) -> roll. 관절명은 HomeReturnConfig의
	# '{관절명}_deg' 필드와 짝이다.
	arm_joint_specs = (
		('shoulder_pan', 1),
		('shoulder_lift', 2),
		('elbow_flex', 3),
		('wrist_flex', 4),
		('wrist_roll', 5),
	)
	gripper_motor_id = 6
	head_motor_ids = (7, 8)
