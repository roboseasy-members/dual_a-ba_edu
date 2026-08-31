"""양팔 로봇(팔로워) 클래스 모음.

주의: lerobot make_robot_from_config는 config 클래스가 정의된 모듈의
**부모 패키지(이 파일)**에서 'Config'를 뗀 클래스명을 찾는다. 두 번째
탐색 후보는 클래스명.lower()라 언더스코어가 빠져 우리 파일명과 안
맞으므로, 여기의 재노출이 없으면 로봇 생성이 ImportError로 실패한다.
새 로봇을 추가하면 반드시 여기서 재노출할 것.
"""

from .bi_follower_base import BiFollowerBase, BiFollowerBaseConfig
from .bi_follower_client import (
	BiFollowerClient,
	BiFollowerClientConfig,
	BiSo101Client,
	BiSo101ClientConfig,
	BiSo102Client,
	BiSo102ClientConfig,
)
from .bi_so101_follower import BiSo101Follower, BiSo101FollowerConfig
from .bi_so102_follower import BiSo102Follower, BiSo102FollowerConfig

__all__ = [
	'BiFollowerBase',
	'BiFollowerBaseConfig',
	'BiFollowerClient',
	'BiFollowerClientConfig',
	'BiSo101Follower',
	'BiSo101Client',
	'BiSo101ClientConfig',
	'BiSo101FollowerConfig',
	'BiSo102Follower',
	'BiSo102Client',
	'BiSo102ClientConfig',
	'BiSo102FollowerConfig',
]
