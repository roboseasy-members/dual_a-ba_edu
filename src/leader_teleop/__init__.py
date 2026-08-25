"""
leader_teleop
=============
양팔 SO-101/SO-102 리더암 텔레옵 + LeRobot 데이터 수집(record) 패키지.

- 텔레옵 입력: 물리 리더암(SO-101/SO-102 leader, 1~2대). 리더암을 손으로
	움직이면 팔로워암이 관절 단위로 따라간다.
- 로봇: 실기 lerobot의 Robot을 상속한 양팔+머리 통합 로봇
	(bus1=왼팔 6모터 + 머리 2모터[카메라 pan/tilt], bus2=오른팔 6모터).
"""

# 공개 API 편의 재노출. lerobot의 클래스 탐색은 robots/__init__.py가
# 담당한다 (config 정의 모듈의 부모 패키지에서 찾기 때문).
from .robots import (
	BiSo101Follower,
	BiSo101FollowerConfig,
	BiSo102Follower,
	BiSo102FollowerConfig,
)

__all__ = [
	'BiSo101Follower',
	'BiSo101FollowerConfig',
	'BiSo102Follower',
	'BiSo102FollowerConfig',
]
