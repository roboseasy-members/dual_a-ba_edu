"""
추론 진입점 (lerobot-rollout 래퍼)
==================================
학습한 정책을 로봇에서 실행합니다. lerobot의 `lerobot-rollout`을 그대로
쓰되, 이 레포의 로봇 타입(`bi_so101_follower`/`bi_so102_follower`와
원격 `bi_so101_client`/`bi_so102_client`)을 먼저 등록해 `--robot.type`으로
고를 수 있게 합니다. `lerobot-rollout` 명령을 직접 쓰면 이 타입들을
모르므로(플러그인 탐색은 `lerobot_robot_*` 패키지만), 이 모듈로 실행합니다.

정책이 `smolvla_base`에서 파인튜닝됐으면 학습 때와 같은 `--rename_map`을
줘야 합니다 — 정책은 카메라를 `camera1/2/3`으로 기대합니다.

실행 예 (PC, 파이 host 실행 중):

	PYTHONPATH=src python -m leader_teleop.rollout \\
		--robot.type=bi_so102_client --robot.remote_ip=<파이 IP> \\
		--robot.cameras='{"cam_top": {"type": "opencv", "index_or_path": 0,
			"width": 640, "height": 480, "fps": 30}, ...}' \\
		--policy.path=outputs/<학습 출력>/checkpoints/last/pretrained_model \\
		--rename_map='{"observation.images.cam_top": "observation.images.camera1", ...}' \\
		--task="Put the red die on the yellow cloth." --duration=60
"""

from lerobot.scripts.lerobot_rollout import rollout
from lerobot.utils.import_utils import register_third_party_plugins

from . import robots  # noqa: F401  - --robot.type 선택지 등록 (임포트 부작용)


def main() -> None:
	"""서드파티 플러그인과 레포 로봇 타입을 등록한 뒤 rollout을 실행한다."""
	register_third_party_plugins()
	rollout()


if __name__ == '__main__':
	main()
