"""
host ↔ client 관측/명령 직렬화
==============================
라즈베리파이(host)와 PC(client) 사이를 오가는 메시지 포맷을 한 곳에
모아둔 모듈입니다. 양쪽이 같은 함수를 쓰므로 포맷이 어긋날 일이 없습니다.

	- 관측(host -> client): 관절 상태(float)와 카메라 프레임(JPEG를
	  base64 문자열로), 그리고 메타(송신 시각 `_t`, 순번 `_seq`)를 JSON
	  객체 하나에 담는다. 메타 키는 밑줄로 시작해 관절 키와 구분된다.
	- 명령(client -> host): '{모터}.pos' -> float 딕셔너리를 그대로 JSON.

전송 계층(ZMQ)은 host.py / robots/bi_follower_client.py가 담당합니다.
"""

import base64
import json
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any, Final

import cv2
import numpy as np


# ZMQ 기본 포트 (host가 bind, client가 connect).
DEFAULT_PORT_ZMQ_CMD: Final[int] = 5555  # client -> host 명령
DEFAULT_PORT_ZMQ_OBSERVATIONS: Final[int] = 5556  # host -> client 관측
# 카메라 프레임 JPEG 품질 (0~100). 카메라 3대·WiFi 기준 80이 무난.
DEFAULT_JPEG_QUALITY: Final[int] = 80
# 관측 메시지 메타 키 (관절 상태 키와 구분되게 밑줄로 시작).
TIMESTAMP_KEY: Final[str] = '_t'  # host 송신 시각 (time.time)
SEQUENCE_KEY: Final[str] = '_seq'  # host 송신 순번 (1부터 증가)
META_KEYS: Final[frozenset[str]] = frozenset({TIMESTAMP_KEY, SEQUENCE_KEY})


@dataclass
class DecodedObservation:
	"""decode_observation() 결과.

	Attributes:
		state: 관절 상태 ('{모터}.pos' -> 값).
		frames: 카메라 이름 -> 디코딩된 프레임 (실패한 프레임은 빠짐).
		timestamp: host 송신 시각 (없으면 None).
		sequence: host 송신 순번 (없으면 None).
	"""

	state: dict[str, float] = field(default_factory=dict)
	frames: dict[str, np.ndarray] = field(default_factory=dict)
	timestamp: float | None = None
	sequence: int | None = None


def encode_observation(
	observation: dict[str, Any],
	camera_keys: Iterable[str],
	jpeg_quality: int = DEFAULT_JPEG_QUALITY,
	timestamp: float | None = None,
	sequence: int | None = None,
) -> str:
	"""로봇 관측을 JSON 문자열로 직렬화한다.

	카메라 키의 값(ndarray)은 JPEG로 압축해 base64 문자열로, 나머지
	키의 값은 float로 담는다. 인코딩에 실패한 프레임은 빈 문자열이다.

	Args:
		observation: robot.get_observation() 결과.
		camera_keys: 관측 중 카메라 프레임인 키 목록.
		jpeg_quality: JPEG 품질 (0~100).
		timestamp: 송신 시각. 주면 `_t` 메타로 실린다.
		sequence: 송신 순번. 주면 `_seq` 메타로 실린다.

	Returns:
		JSON 문자열.
	"""
	camera_key_set = set(camera_keys)
	payload: dict[str, Any] = {}
	for key, value in observation.items():
		if key in camera_key_set:
			is_ok, buffer = cv2.imencode(
				'.jpg', value, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
			)
			payload[key] = (
				base64.b64encode(buffer).decode('ascii') if is_ok else ''
			)
		else:
			payload[key] = float(value)
	if timestamp is not None:
		payload[TIMESTAMP_KEY] = float(timestamp)
	if sequence is not None:
		payload[SEQUENCE_KEY] = int(sequence)
	return json.dumps(payload)


def decode_observation(
	message: str, camera_keys: Iterable[str],
) -> DecodedObservation:
	"""JSON 관측 문자열을 관절 상태·카메라 프레임·메타로 되돌린다.

	카메라 키로 선언되지 않은 문자열 값(양쪽 --robot.cameras가 어긋난
	경우)은 무시한다 — 루프가 죽지 않게 하기 위해서다.

	Args:
		message: encode_observation()이 만든 JSON 문자열.
		camera_keys: 카메라 프레임으로 해석할 키 목록.

	Returns:
		DecodedObservation.

	Raises:
		ValueError: 메시지가 JSON이 아니거나 JSON 객체(dict)가 아닐 때
			(json.JSONDecodeError는 ValueError의 서브클래스).
	"""
	camera_key_set = set(camera_keys)
	payload = json.loads(message)
	if not isinstance(payload, dict):
		raise ValueError(
			f'observation payload must be a JSON object, got '
			f'{type(payload).__name__}'
		)
	decoded = DecodedObservation()
	for key, value in payload.items():
		if key in META_KEYS:
			continue
		if key in camera_key_set:
			frame = _decode_frame(value)
			if frame is not None:
				decoded.frames[key] = frame
		elif isinstance(value, (int, float)):
			decoded.state[key] = float(value)
	timestamp = payload.get(TIMESTAMP_KEY)
	if isinstance(timestamp, (int, float)):
		decoded.timestamp = float(timestamp)
	sequence = payload.get(SEQUENCE_KEY)
	if isinstance(sequence, int):
		decoded.sequence = sequence
	return decoded


def _decode_frame(image_b64: Any) -> np.ndarray | None:
	"""base64 JPEG 문자열을 ndarray로 디코딩한다 (실패 시 None)."""
	if not isinstance(image_b64, str) or not image_b64:
		return None
	try:
		jpeg_bytes = base64.b64decode(image_b64)
	except (TypeError, ValueError):
		return None
	if not jpeg_bytes:
		return None  # cv2.imdecode는 빈 버퍼에서 예외를 낸다
	buffer = np.frombuffer(jpeg_bytes, dtype=np.uint8)
	return cv2.imdecode(buffer, cv2.IMREAD_COLOR)
