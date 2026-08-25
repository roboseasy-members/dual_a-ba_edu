"""
설정 모듈
=========
여러 모듈이 공유하는 튜닝 파라미터를 한 곳에 모아둔 모듈입니다. 값을
바꾸고 싶으면 항상 이 파일만 보면 됩니다 (다른 모듈에는 숫자 상수를
흩뿌리지 않습니다).

담고 있는 것: 그리퍼 키 상수, 카메라 헤드 홈 각도 상수(헤드 제어 설정
자체는 head/head_modes.py의 HeadControlConfig), 홈포즈
복귀(HomeReturnConfig). 직접 실행(teleoperate)과 record 두 경로 모두
동일한 draccus CLI(--camera_head.* / --home_return.*)로 이 파일의
기본값을 실행 시 오버라이드할 수 있습니다.

주의(draccus 함정): CLI로 파싱되는 config 클래스는 Google 스타일
`Attributes:` docstring을 쓰면 파싱이 깨집니다. 한 줄 docstring +
필드 인라인 주석으로 문서화하세요.
"""

from dataclasses import dataclass
from typing import Final


GRIPPER_NAME: Final[str] = 'gripper'


# ============================================================
# 카메라 헤드 홈 각도 (HeadControlConfig와 HomeReturnConfig가 공유)
# ============================================================
HEAD_PAN_HOME_DEG: Final[float] = 0.0  # pan(head_motor_1) 홈 각도 (deg)
HEAD_TILT_HOME_DEG: Final[float] = 30.0  # tilt(head_motor_2) 홈 각도 (deg)


@dataclass
class HomeReturnConfig:
	"""종료 시 팔/머리 홈포즈(수그린 자세) 복귀 설정 (draccus CLI 파싱 대상)."""

	is_enabled: bool = True  # 종료 시 홈포즈 복귀 on/off
	duration_s: float = 3.0  # 현재 자세 -> 홈포즈 보간 이동 시간 (s)
	fps: float = 50.0  # 보간 전송 주기 (Hz)
	settle_s: float = 0.5  # 도달 후 토크 해제 전 유지 시간 (s)
	# 팔 홈 관절각 (양팔 공통, deg). lerobot 공식 예제 isaac_teleop_to_so101
	# 의 RESET_ORIGIN_DEG(팔꿈치/손목이 접힌 수그린 자세) 기반 + 실기 튜닝
	# 반영(2026-07-23). 미세조정은 capture_home_pose 유틸리티로 측정해 갱신.
	shoulder_pan_deg: float = 0.0  # 어깨 pan 홈 (deg)
	shoulder_lift_deg: float = -103.0  # 어깨 lift 홈 (deg)
	elbow_flex_deg: float = 97.0  # 팔꿈치 홈 (deg)
	wrist_flex_deg: float = 60.0  # 손목 pitch 홈 (deg)
	wrist_yaw_deg: float = 0.0  # 손목 yaw 홈 (deg, SO-102 전용 - SO-101 무시)
	wrist_roll_deg: float = 0.0  # 손목 roll 홈 (deg)
	gripper_pos: float = 2.0  # 그리퍼 홈 (0~100, 0=닫힘)
	head_pan_deg: float = HEAD_PAN_HOME_DEG  # 머리 pan 홈 (deg)
	head_tilt_deg: float = HEAD_TILT_HOME_DEG  # 머리 tilt 홈 (deg)


# 로봇 하드웨어 설정은 robots/bi_follower_base.py의
# BiFollowerBaseConfig(--robot.*), 직접 실행 진입점 설정은 app.py의
# TeleopAppConfig가 담당한다 (두 경로 CLI 통일).
