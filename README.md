# dual_a-ba_edu

**양팔 SO-101 리더암 텔레옵 + LeRobot 데이터 수집(record)**
레포지토리입니다. 리더암(사람이 손으로 움직이는 팔)을 움직이면
팔로워암(로봇)이 관절 단위로 그대로 따라가고, 그 동작을 LeRobot
데이터셋으로 녹화할 수 있습니다.

## 타깃 하드웨어

| 구성 | 내용 | 전원 |
|------|------|------|
| 팔로워 (로봇) | 양팔 SO-101, 각 6모터 (id 1~6) | 12V |
| 카메라 헤드 (선택) | pan/tilt 2모터 (id 7/8), **왼팔과 같은 버스** | 12V |
| 리더암 (조종) | SO-101 leader 1~2대, 각 6모터 (id 1~6) | 5V |

- `bus1`(left_arm_port) = 왼팔 6모터 + (장착 시) 머리 2모터,
  `bus2`(right_arm_port) = 오른팔 6모터.
- SO-102(손목 yaw 추가 7모터)도 지원합니다 — 아래 명령에서
  `--robot.type=bi_so102_follower` + `--teleop.type=bi_so102_leader`로
  바꾸면 됩니다 (로봇/리더는 **같은 세대끼리만** 조합, 시작 시 검증).

## 설치

Python 3.12 기준이며, conda, venv 등 가상환경 사용을 권장합니다.

```bash
git clone <레포 URL> && cd dual_a-ba_edu
pip install -r requirements.txt   # lerobot==0.6.0 포함
```

## 포트 확인

USB를 꽂은 뒤 어떤 포트가 어느 팔인지 먼저 확인합니다 (읽기 전용 점검
— 모터가 움직이지 않습니다):

```bash
PYTHONPATH=src python -m leader_teleop.scripts.check_robot
```

포트별로 팔로워 bus1(머리 포함)/팔로워 팔/리더암(5V)을 분류해 실행
플래그를 제안해 줍니다. 포트 번호는 재연결 시 뒤바뀔 수 있으니 주의.

## 실행 (텔레옵만, 녹화 없음)

```bash
# 리더암 2대로 양팔 조종
PYTHONPATH=src python -m leader_teleop.teleoperate \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM2 \
	--teleop.right_arm_port=/dev/ttyACM3

# 리더암 1대로 왼팔만 (오른팔은 시작 자세 유지)
PYTHONPATH=src python -m leader_teleop.teleoperate \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.mode=left --teleop.left_arm_port=/dev/ttyACM2
```

- **첫 실행 시 캘리브레이션**을 진행합니다 (터미널 안내를 따라 중립
  자세 → 전체 가동범위 순서로 움직이면 파일로 저장, 이후 재사용).
- 종료는 `Ctrl+C` — 팔이 홈포즈(수그린 자세)로 복귀한 뒤 토크가
  풀립니다 (`--home_return.is_enabled=false`로 끌 수 있음).
- 리더암은 손을 놓으면 팔로워가 그 자리에 멈춥니다.

## 데이터 수집 (record)

텔레옵과 **같은 인자 체계**에 `--dataset.*`가 더해집니다. 카메라는
`--robot.cameras`로 등록합니다 (lerobot 표준).

```bash
PYTHONPATH=src python -m leader_teleop.record \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}' \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM2 \
	--teleop.right_arm_port=/dev/ttyACM3 \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.single_task='물건을 집어 상자에 넣는다' \
	--dataset.num_episodes=10 --dataset.push_to_hub=false \
	--display_data=true
```

에피소드 제어 (lerobot 표준 키보드): `→` 다음 에피소드로 조기 종료,
`←` 현재 에피소드 재녹화, `Esc` 녹화 전체 종료. record 중 `Ctrl+C`는
진행 중 에피소드를 버리므로 종료는 `Esc`를 쓰세요.

## 카메라 헤드 (선택)

머리 pan/tilt 모터가 장착된 구성이면 `--camera_head.mode`로 제어합니다.
기본 `none`(미장착)이며, 이때 머리 모터는 검색조차 하지 않습니다.

| 모드 | 동작 |
|------|------|
| `none` (기본) | 머리 미장착 구성 — 머리 키가 데이터셋에서도 빠짐 |
| `fixed` | 시작 시 고정각(기본 홈각)으로 이동 후 유지 |
| `keyboard` | `a/d`=pan, `w/s`=tilt, `h`=홈 (record 방향키와 안 겹침) |

## 홈포즈 조정

종료 시 복귀하는 홈포즈 관절각은 `src/leader_teleop/config.py`의
`HomeReturnConfig`에 있습니다. 내 로봇에 맞는 값을 측정하려면:

```bash
PYTHONPATH=src python -m leader_teleop.scripts.capture_home_pose \
	--left-arm-port /dev/ttyACM0 --right-arm-port /dev/ttyACM1
```

토크가 풀린 상태에서 원하는 자세로 팔을 잡아주면 config에 붙여넣을
값을 출력합니다.

## 코드 구조

| 역할 | 파일 |
|------|------|
| 진입점 (직접 텔레옵 / 데이터 수집) | `teleoperate.py` / `record.py` |
| 조립·연결·제어 루프·정리 | `app.py` |
| 공유 설정 (홈포즈 등) | `config.py` |
| 양팔+머리 팔로워 로봇 (공통/세대 명세) | `robots/bi_follower_base.py` + `bi_so101_follower.py`/`bi_so102_follower.py` |
| 양팔 리더암 텔레옵 (공통/세대 명세) | `teleoperators/bi_leader_base.py` + `bi_so101_leader.py`/`bi_so102_leader.py` |
| 팔 + 머리 합성 텔레옵 | `teleoperators/combined_teleop.py` |
| 카메라 헤드 모드 | `head/head_modes.py` |
| 연결 재시도 (간헐 통신 글리치 흡수) | `hardware_retry.py` |
| 하드웨어 점검 / 홈포즈 캡처 | `scripts/check_robot.py` / `scripts/capture_home_pose.py` |

## 안전 주의

- 실행 전 로봇 주변을 정리하고, 언제든 `Ctrl+C`로 멈출 수 있게
  터미널에 손을 두세요.
- 전압 에러가 반복되면 전원을 점검하세요 — SO-101 **리더암은 5V,
  팔로워암은 12V**입니다. 데이지체인 커넥터도 다시 꽂아 보세요.
- 리더암과 팔로워암의 좌/우를 바꿔 꽂으면 팔이 교차 동작합니다.
  실행 직후 천천히 움직여 좌/우 대응을 먼저 확인하세요.
