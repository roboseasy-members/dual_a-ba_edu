"""
SO-102 양팔 리더암
==================
공통 구현은 bi_leader_base 참조. SO-102 리더는 손목에 yaw 축이 추가된
7모터 팔입니다 (팔로워와 동일 구성, 7=그리퍼). 캘리브레이션 파일은
teleoperators/bi_so102_leader/{id}_{side}.json — SO-101 파일과 디렉터리
부터 분리되어 섞이지 않습니다.
"""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

from .bi_leader_base import BiLeaderBase, BiLeaderBaseConfig


@TeleoperatorConfig.register_subclass('bi_so102_leader')
@dataclass
class BiSo102LeaderConfig(BiLeaderBaseConfig):
	"""SO-102 양팔 리더암 설정 (공통 필드는 BiLeaderBaseConfig)."""


class BiSo102Leader(BiLeaderBase):
	"""SO-102 리더암 (각 7모터, id 1~7) 텔레오퍼레이터."""

	config_class = BiSo102LeaderConfig
	name = 'bi_so102_leader'

	# 짝 팔로워(robots/bi_so102_follower.py)의 명세와 동일해야 한다.
	arm_joint_specs = (
		('shoulder_pan', 1),
		('shoulder_lift', 2),
		('elbow_flex', 3),
		('wrist_flex', 4),
		('wrist_yaw', 5),
		('wrist_roll', 6),
	)
	gripper_motor_id = 7
