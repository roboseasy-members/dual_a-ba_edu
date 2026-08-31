"""
host ↔ client 관측/명령 직렬화
==============================
라즈베리파이(host)와 PC(client) 사이를 오가는 메시지 포맷을 한 곳에
모아둔 모듈입니다. 양쪽이 같은 함수를 쓰므로 포맷이 어긋날 일이 없습니다.

	- 관측(host -> client): 관절 상태(float)와 카메라 프레임(JPEG를
	  base64 문자열로)을 JSON 객체 하나에 담는다.
	- 명령(client -> host): '{모터}.pos' -> float 딕셔너리를 그대로 JSON.

전송 계층(ZMQ)은 host.py / robots/bi_follower_client.py가 담당합니다.
"""

import base64
import json
from collections.abc import Iterable
from typing import Any, Final

import cv2
import numpy as np


# ZMQ 기본 포트 (host가 bind, client가 connect).
DEFAULT_PORT_ZMQ_CMD: Final[int] = 5555  # client -> host 명령
DEFAULT_PORT_ZMQ_OBSERVATIONS: Final[int] = 5556  # host -> client 관측
# 카메라 프레임 JPEG 품질 (0~100). 카메라 3대·WiFi 기준 80이 무난.
DEFAULT_JPEG_QUALITY: Final[int] = 80


def encode_observation(
	observation: dict[str, Any],
	camera_keys: Iterable[str],
	jpeg_quality: int = DEFAULT_JPEG_QUALITY,
) -> str:
	"""로봇 관측을 JSON 문자열로 직렬화한다.

	카메라 키의 값(ndarray)은 JPEG로 압축해 base64 문자열로, 나머지
	키의 값은 float로 담는다. 인코딩에 실패한 프레임은 빈 문자열이다.

	Args:
		observation: robot.get_observation() 결과.
		camera_keys: 관측 중 카메라 프레임인 키 목록.
		jpeg_quality: JPEG 품질 (0~100).

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
	return json.dumps(payload)


def decode_observation(
	message: str, camera_keys: Iterable[str],
) -> tuple[dict[str, float], dict[str, np.ndarray]]:
	"""JSON 관측 문자열을 관절 상태와 카메라 프레임으로 되돌린다.

	Args:
		message: encode_observation()이 만든 JSON 문자열.
		camera_keys: 카메라 프레임으로 해석할 키 목록.

	Returns:
		(관절 상태 딕셔너리, 카메라 이름 -> 프레임 ndarray 딕셔너리).
		디코딩에 실패한 프레임은 두 번째 딕셔너리에서 빠지고, 카메라
		키로 선언되지 않은 문자열 값은 무시된다.

	Raises:
		json.JSONDecodeError: 메시지가 JSON이 아닐 때.
	"""
	camera_key_set = set(camera_keys)
	payload = json.loads(message)
	state: dict[str, float] = {}
	frames: dict[str, np.ndarray] = {}
	for key, value in payload.items():
		if key in camera_key_set:
			frame = _decode_frame(value)
			if frame is not None:
				frames[key] = frame
		elif isinstance(value, (int, float)):
			state[key] = float(value)
		# 그 외(문자열 = 선언하지 않은 카메라의 프레임)는 무시한다 -
		# 양쪽 --robot.cameras가 어긋나도 루프가 죽지 않게.
	return state, frames


def _decode_frame(image_b64: str) -> np.ndarray | None:
	"""base64 JPEG 문자열을 ndarray로 디코딩한다 (실패 시 None)."""
	if not image_b64:
		return None
	try:
		jpeg_bytes = base64.b64decode(image_b64)
	except (TypeError, ValueError):
		return None
	buffer = np.frombuffer(jpeg_bytes, dtype=np.uint8)
	return cv2.imdecode(buffer, cv2.IMREAD_COLOR)
