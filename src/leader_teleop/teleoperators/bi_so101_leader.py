"""
SO-101 양팔 리더암
==================
공통 구현은 bi_leader_base 참조. 이 파일은 SO-101 리더 하드웨어 명세
(관절 5 + 그리퍼 id 6)만 선언합니다. 캘리브레이션 파일은
teleoperators/bi_so101_leader/{id}_{side}.json.
"""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

from .bi_leader_base import BiLeaderBase, BiLeaderBaseConfig


@TeleoperatorConfig.register_subclass('bi_so101_leader')
@dataclass
class BiSo101LeaderConfig(BiLeaderBaseConfig):
	"""SO-101 양팔 리더암 설정 (공통 필드는 BiLeaderBaseConfig)."""


class BiSo101Leader(BiLeaderBase):
	"""SO-101 리더암 (각 6모터, id 1~6) 텔레오퍼레이터."""

	config_class = BiSo101LeaderConfig
	name = 'bi_so101_leader'

	# 짝 팔로워(robots/bi_so101_follower.py)의 명세와 동일해야 한다.
	arm_joint_specs = (
		('shoulder_pan', 1),
		('shoulder_lift', 2),
		('elbow_flex', 3),
		('wrist_flex', 4),
		('wrist_roll', 5),
	)
	gripper_motor_id = 6
