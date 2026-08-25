"""
실행 진입점 (직접 텔레옵, 녹화 없음)
====================================
record와 동일한 draccus CLI 체계(`--robot.*` / `--teleop.*` /
`--camera_head.*` / `--home_return.*`)로 양팔 리더암 텔레옵만 실행합니다
(데이터셋/rerun 관련 인자만 없음). 서브필드를 오버라이드할 때는
draccus 규칙상 `--robot.type` / `--teleop.type`을 함께 지정해야 합니다.

실행 예:

	# 리더암 2대 (기본: bi_so101_follower + bi_so101_leader)
	PYTHONPATH=src python -m leader_teleop.teleoperate \\
		--robot.type=bi_so101_follower --robot.id=bi_so101_follower \\
		--robot.left_arm_port=/dev/ttyACM0 \\
		--robot.right_arm_port=/dev/ttyACM1 \\
		--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \\
		--teleop.left_arm_port=/dev/ttyACM2 \\
		--teleop.right_arm_port=/dev/ttyACM3

	# 왼팔 리더암 1대만 (오른팔은 시작 자세 유지) + 키보드 헤드
	PYTHONPATH=src python -m leader_teleop.teleoperate \\
		--robot.type=bi_so101_follower --robot.id=bi_so101_follower \\
		--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \\
		--teleop.mode=left --teleop.left_arm_port=/dev/ttyACM2 \\
		--camera_head.mode=keyboard
"""

import logging
from dataclasses import asdict
from pprint import pformat

from lerobot.configs import parser
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

from .app import TeleopApp, TeleopAppConfig


@parser.wrap()
def teleoperate(cfg: TeleopAppConfig) -> None:
	"""파싱된 설정으로 텔레옵 앱을 실행한다.

	Args:
		cfg: CLI에서 파싱된 실행 설정.
	"""
	init_logging()
	logging.info(pformat(asdict(cfg)))
	app = TeleopApp(cfg)
	app.setup()
	app.run()


def main() -> None:
	"""서드파티 플러그인 등록 후 텔레옵을 시작한다.

	제어 루프 중의 Ctrl+C는 TeleopApp이 정리(홈포즈 복귀/토크 해제)까지
	처리하므로, 여기서는 setup 도중의 Ctrl+C만 조용히 마무리한다.
	"""
	register_third_party_plugins()
	try:
		teleoperate()
	except KeyboardInterrupt:
		print('\n[system] Shutting down.')


if __name__ == '__main__':
	main()
