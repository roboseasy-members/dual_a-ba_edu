"""
데이터 수집(lerobot record) + 카메라 헤드 진입점
================================================
`lerobot record`와 동일한 인자/동작(에피소드 녹화, 데이터셋 저장, rerun
시각화, 방향키 에피소드 제어)에 카메라 헤드 제어를 더한 스크립트다.
팔 조종은 물리 리더암(--teleop.type=bi_so101_leader/bi_so102_leader)이다.
에피소드 루프의 핵심인 `record_loop`은 lerobot 설치본의 것을 수정 없이
그대로 재사용한다.

우리 로봇(BiSo101Follower)은 머리 모터가 native라 별도의 카메라 헤드
우회 래퍼가 필요 없다. 머리 목표각을 텔레옵 action에 실어 보내면
record_loop의 robot.send_action()이 그대로 구동한다.

선택 (CLI):
	--teleop.type=bi_so101_leader  # SO-101 리더암 (bi_so102_leader도 가능)
	--camera_head.mode=none|fixed|keyboard

사용 예:

	PYTHONPATH=src python -m leader_teleop.record \\
		--robot.type=bi_so101_follower \\
		--robot.left_arm_port=/dev/ttyACM0 --robot.right_arm_port=/dev/ttyACM1 \\
		--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \\
		--teleop.left_arm_port=/dev/ttyACM2 \\
		--teleop.right_arm_port=/dev/ttyACM3 \\
		--dataset.repo_id=roboseasy/pick_place \\
		--dataset.single_task='물건을 집어 상자에 넣는다' \\
		--dataset.num_episodes=10 --dataset.push_to_hub=false \\
		--display_data=true
"""

import logging
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Any

from lerobot.common.control_utils import (
	sanity_check_dataset_robot_compatibility,
)
from lerobot.configs import parser
from lerobot.datasets import (
	LeRobotDataset,
	VideoEncodingManager,
	aggregate_pipeline_dataset_features,
	create_initial_features,
)
from lerobot.processor import (
	ObservationProcessorStep,
	RobotProcessorPipeline,
	make_default_processors,
)
from lerobot.processor.converters import (
	observation_to_transition,
	transition_to_observation,
)
from lerobot.processor.pipeline import PipelineFeatureType
from lerobot.scripts.lerobot_record import RecordConfig, record_loop
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import (
	init_visualization,
	shutdown_visualization,
)

# app 임포트가 robots/teleoperators 패키지 임포트를 겸하므로 draccus
# 서브클래스가 전부 등록되어 --robot.type / --teleop.type 파싱이 된다.
from .app import TeleopApp, TeleopStackConfig
from .head.head_modes import PAN_MOTOR_KEY, TILT_MOTOR_KEY


class ExcludeObservationKeysStep(ObservationProcessorStep):
	"""관측에서 지정 키를 제거하는 파이프라인 스텝.

	헤드 모드 none일 때 머리 모터 키를 제거하는 데 쓴다.
	transform_features가 데이터셋 feature 계산에도 같은 제외를
	적용하므로, 데이터셋 observation.state와 rerun 표시에서 함께 빠진다.
	"""

	def __init__(self, excluded_keys: tuple[str, ...]) -> None:
		self._excluded_keys = frozenset(excluded_keys)

	def observation(self, observation: dict[str, Any]) -> dict[str, Any]:
		"""제외 키를 뺀 관측 딕셔너리를 반환한다."""
		return {
			key: value
			for key, value in observation.items()
			if key not in self._excluded_keys
		}

	def transform_features(
		self,
		features: dict[PipelineFeatureType, dict[str, Any]],
	) -> dict[PipelineFeatureType, dict[str, Any]]:
		"""observation feature에서도 같은 키를 제외한다."""
		features[PipelineFeatureType.OBSERVATION] = {
			key: value
			for key, value in features[
				PipelineFeatureType.OBSERVATION
			].items()
			if key not in self._excluded_keys
		}
		return features


@dataclass
class RecordWithHeadConfig(TeleopStackConfig, RecordConfig):
	"""lerobot RecordConfig에 카메라 헤드 등 조립 공통 설정을 더한 config.

	추가 필드(--camera_head.* / --home_return.*)는 직접 실행과 공유하는
	TeleopStackConfig가 정의한다.
	"""


@parser.wrap()
def record(cfg: RecordWithHeadConfig) -> LeRobotDataset:
	"""카메라 헤드와 함께 데이터 수집을 실행한다.

	lerobot record()의 흐름을 그대로 따르되, 리더암/카메라 헤드 조립과
	영점 초기화 훅을 끼워 넣는다. 에피소드 루프(record_loop)는
	lerobot 것을 수정 없이 사용한다.

	Args:
		cfg: CLI에서 파싱된 실행 설정.

	Returns:
		수집이 끝난 LeRobotDataset.
	"""
	init_logging()
	logging.info(pformat(asdict(cfg)))
	if cfg.display_data:
		init_visualization(
			cfg.display_mode,
			session_name='recording',
			ip=cfg.display_ip,
			port=cfg.display_port,
		)
	display_compressed_images = (
		True
		if (
			cfg.display_data
			and cfg.display_ip is not None
			and cfg.display_port is not None
		)
		else cfg.display_compressed_images
	)

	# 로봇/팔 텔레옵/헤드 조립은 직접 실행과 같은 TeleopApp이 담당한다
	# (record_loop만 여기서 다르다). 연결·영점 초기화는 app.setup()이 한다.
	app = TeleopApp(cfg)
	robot = app.robot
	teleop = app.teleop

	(
		teleop_action_processor,
		robot_action_processor,
		robot_observation_processor,
	) = make_default_processors()

	# 헤드 모드 none이면 머리 2키를 action/observation 양쪽에서 뺀다.
	# action: 아래 feature 필터 + CombinedTeleop(head=None)이 팔 12키만
	# 출력해 feature의 names와 실제 키가 일치한다. observation: 프로세서
	# 스텝이 관측값과 feature에서 함께 제거해 데이터셋
	# observation.state와 rerun 표시에 모두 반영된다.
	action_features = dict(robot.action_features)
	if teleop.head is None:
		action_features.pop(PAN_MOTOR_KEY, None)
		action_features.pop(TILT_MOTOR_KEY, None)
		robot_observation_processor = RobotProcessorPipeline(
			steps=[
				ExcludeObservationKeysStep(
					(PAN_MOTOR_KEY, TILT_MOTOR_KEY)
				),
			],
			to_transition=observation_to_transition,
			to_output=transition_to_observation,
		)

	dataset_features = combine_feature_dicts(
		aggregate_pipeline_dataset_features(
			pipeline=teleop_action_processor,
			initial_features=create_initial_features(
				action=action_features
			),
			use_videos=cfg.dataset.video,
		),
		aggregate_pipeline_dataset_features(
			pipeline=robot_observation_processor,
			initial_features=create_initial_features(
				observation=robot.observation_features
			),
			use_videos=cfg.dataset.video,
		),
	)

	dataset = None
	listener = None

	try:
		num_cameras = len(robot.cameras) if hasattr(robot, 'cameras') else 0
		if cfg.resume:
			dataset = LeRobotDataset.resume(
				cfg.dataset.repo_id,
				root=cfg.dataset.root,
				batch_encoding_size=cfg.dataset.video_encoding_batch_size,
				rgb_encoder=cfg.dataset.rgb_encoder,
				depth_encoder=cfg.dataset.depth_encoder,
				encoder_threads=cfg.dataset.encoder_threads,
				streaming_encoding=cfg.dataset.streaming_encoding,
				encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
				image_writer_processes=(
					cfg.dataset.num_image_writer_processes
					if num_cameras > 0 else 0
				),
				image_writer_threads=(
					cfg.dataset.num_image_writer_threads_per_camera
					* num_cameras
					if num_cameras > 0 else 0
				),
			)
			sanity_check_dataset_robot_compatibility(
				dataset, robot, cfg.dataset.fps, dataset_features
			)
		else:
			repo_name = cfg.dataset.repo_id.split('/', 1)[-1]
			if repo_name.startswith('eval_'):
				raise ValueError(
					"'eval_'로 시작하는 데이터셋 이름은 정책 평가용으로 "
					'예약돼 있습니다. 데이터 수집에는 다른 이름을 쓰세요.'
				)
			cfg.dataset.stamp_repo_id()
			dataset = LeRobotDataset.create(
				cfg.dataset.repo_id,
				cfg.dataset.fps,
				root=cfg.dataset.root,
				robot_type=robot.name,
				features=dataset_features,
				use_videos=cfg.dataset.video,
				image_writer_processes=(
					cfg.dataset.num_image_writer_processes
				),
				image_writer_threads=(
					cfg.dataset.num_image_writer_threads_per_camera
					* num_cameras
				),
				batch_encoding_size=cfg.dataset.video_encoding_batch_size,
				rgb_encoder=cfg.dataset.rgb_encoder,
				depth_encoder=cfg.dataset.depth_encoder,
				encoder_threads=cfg.dataset.encoder_threads,
				streaming_encoding=cfg.dataset.streaming_encoding,
				encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
			)

		# 연결·초기화 시퀀스(로봇 -> 텔레옵 -> 영점 초기화)는 직접
		# 실행과 동일.
		app.setup()

		listener, events = init_keyboard_listener()

		with VideoEncodingManager(dataset):
			recorded_episodes = 0
			while (
				recorded_episodes < cfg.dataset.num_episodes
				and not events['stop_recording']
			):
				log_say(
					f'Recording episode {dataset.num_episodes}',
					cfg.play_sounds,
				)
				record_loop(
					robot=robot,
					events=events,
					fps=cfg.dataset.fps,
					teleop_action_processor=teleop_action_processor,
					robot_action_processor=robot_action_processor,
					robot_observation_processor=robot_observation_processor,
					teleop=teleop,
					dataset=dataset,
					control_time_s=cfg.dataset.episode_time_s,
					single_task=cfg.dataset.single_task,
					display_data=cfg.display_data,
					display_mode=cfg.display_mode,
					display_compressed_images=display_compressed_images,
				)

				# 환경 리셋 시간 (마지막 에피소드는 생략, 데이터 저장 안 함)
				if not events['stop_recording'] and (
					recorded_episodes < cfg.dataset.num_episodes - 1
					or events['rerecord_episode']
				):
					log_say('Reset the environment', cfg.play_sounds)
					record_loop(
						robot=robot,
						events=events,
						fps=cfg.dataset.fps,
						teleop_action_processor=teleop_action_processor,
						robot_action_processor=robot_action_processor,
						robot_observation_processor=(
							robot_observation_processor
						),
						teleop=teleop,
						control_time_s=cfg.dataset.reset_time_s,
						single_task=cfg.dataset.single_task,
						display_data=cfg.display_data,
						display_mode=cfg.display_mode,
					)

				if events['rerecord_episode']:
					log_say('Re-record episode', cfg.play_sounds)
					events['rerecord_episode'] = False
					events['exit_early'] = False
					dataset.clear_episode_buffer()
					continue

				dataset.save_episode()
				recorded_episodes += 1
	finally:
		log_say('Stop recording', cfg.play_sounds, blocking=True)

		# 실기 안전 우선: 홈포즈 복귀 + 토크 해제 + 텔레옵 종료를 데이터
		# 마무리보다 먼저 한다 (finalize가 디스크/인코더 오류로 죽어도
		# 토크 해제는 이미 끝난 상태. 중복 호출 안전).
		app.shutdown()

		# 주의: LeRobotDataset은 __len__(num_frames)이 있어 truthiness로
		# 검사하면 0프레임 데이터셋이 조용히 건너뛰어진다 - is not None.
		if dataset is not None:
			dataset.finalize()

		if listener is not None:
			listener.stop()

		if cfg.display_data:
			shutdown_visualization(cfg.display_mode)

		if cfg.dataset.push_to_hub:
			if dataset is not None and dataset.num_episodes > 0:
				dataset.push_to_hub(
					tags=cfg.dataset.tags, private=cfg.dataset.private
				)
			else:
				logging.warning('No episodes saved — skipping push to hub')

		log_say('Exiting', cfg.play_sounds)
	return dataset


def main() -> None:
	"""서드파티 플러그인 등록 후 데이터 수집을 시작한다.

	Ctrl+C(KeyboardInterrupt)는 record의 finally에서 정리
	(홈포즈 복귀/토크 해제)가 끝난 뒤 여기까지 올라오므로, 트레이스백
	대신 안내 한 줄만 출력한다.
	"""
	register_third_party_plugins()
	try:
		record()
	except KeyboardInterrupt:
		print('\nInterrupted by user (Ctrl+C) - cleanup already done.')


if __name__ == '__main__':
	main()
