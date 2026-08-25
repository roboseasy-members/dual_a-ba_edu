# dual_a-ba_edu

양팔 SO-101 리더암 텔레옵 + LeRobot 데이터 수집(record) 레포지토리입니다.
리더암을 손으로 움직이면 팔로워암이 관절 단위로 따라갑니다. 그 동작은
LeRobot 데이터셋으로 녹화합니다.

## 타깃 하드웨어

| 구성 | 내용 | 전원 |
|------|------|------|
| 팔로워 (로봇) | 양팔 SO-101, 각 6모터 (id 1~6) | 12V |
| 카메라 헤드 (선택) | pan/tilt 2모터 (id 7/8), 왼팔과 같은 버스 | 12V |
| 리더암 (조종) | SO-101 leader 1~2대, 각 6모터 (id 1~6) | 5V |

- `bus1`(left_arm_port) = 왼팔 6모터 + (장착 시) 머리 2모터.
  `bus2`(right_arm_port) = 오른팔 6모터.
- SO-102(손목 yaw 추가, 7모터)도 지원합니다. 아래 명령에서
  `--robot.type=bi_so102_follower` + `--teleop.type=bi_so102_leader`로
  바꾸면 됩니다. 로봇과 리더는 같은 세대끼리만 조합할 수 있고, 어긋난
  조합은 시작 시점에 에러로 걸러집니다.

## 설치

Python 3.12 기준입니다. conda나 venv 같은 가상환경을 쓰는 편이 좋습니다.

```bash
git clone <레포 URL> && cd dual_a-ba_edu
pip install -r requirements.txt   # lerobot==0.6.0 포함
```

## 포트 확인

USB를 꽂고 어떤 포트가 어느 팔인지부터 확인합니다. 읽기 전용 점검이라
모터는 움직이지 않습니다.

```bash
PYTHONPATH=src python -m leader_teleop.scripts.check_robot
```

포트별로 팔로워 bus1(머리 포함) / 팔로워 팔 / 리더암(5V)을 분류해
실행 플래그를 제안합니다. 포트 번호는 재연결하면 바뀔 수 있습니다.

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

- 첫 실행에서 캘리브레이션이 시작됩니다. 터미널 안내대로 중립 자세,
  전체 가동범위 순서로 움직이면 파일로 저장되고 다음부터 재사용합니다.
- 종료는 `Ctrl+C`. 팔이 홈포즈(수그린 자세)로 복귀한 뒤 토크가
  풀립니다. `--home_return.is_enabled=false`로 끌 수 있습니다.
- 리더암에서 손을 놓으면 팔로워는 그 자리에 멈춥니다.

## 데이터 수집 (record)

텔레옵과 같은 인자 체계에 `--dataset.*`가 더해집니다. 카메라는
`--robot.cameras`로 등록합니다 (lerobot 표준). 아래 예시는 상단 1대 +
양쪽 손목 2대 구성입니다. `index_or_path`는 PC마다 다르니
`v4l2-ctl --list-devices`로 먼저 확인하세요.

```bash
PYTHONPATH=src python -m leader_teleop.record \
	--robot.type=bi_so101_follower --robot.id=bi_so101_follower \
	--robot.left_arm_port=/dev/ttyACM0 \
	--robot.right_arm_port=/dev/ttyACM1 \
	--robot.cameras='{"top": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}, "left_wrist": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}, "right_wrist": {"type": "opencv", "index_or_path": 4, "width": 640, "height": 480, "fps": 30}}' \
	--teleop.type=bi_so101_leader --teleop.id=bi_so101_leader \
	--teleop.left_arm_port=/dev/ttyACM2 \
	--teleop.right_arm_port=/dev/ttyACM3 \
	--dataset.repo_id=<HF계정>/<데이터셋이름> \
	--dataset.single_task='물건을 집어 상자에 넣는다' \
	--dataset.num_episodes=10 --dataset.push_to_hub=false \
	--display_data=true
```

에피소드 제어는 lerobot 표준 키보드를 따릅니다. `→` 다음 에피소드로
조기 종료, `←` 현재 에피소드 재녹화, `Esc` 녹화 전체 종료.
record 중 `Ctrl+C`는 진행 중인 에피소드를 버립니다. 종료는 `Esc`로
하세요.

## 카메라 헤드 (선택)

머리 pan/tilt 모터가 있는 구성이면 `--camera_head.mode`로 제어합니다.
기본은 `none`. 이때는 머리 모터를 검색하지 않습니다.

| 모드 | 동작 |
|------|------|
| `none` (기본) | 머리 미장착 구성 — 머리 키가 데이터셋에서도 빠짐 |
| `fixed` | 시작 시 고정각(기본 홈각)으로 이동 후 유지 |
| `keyboard` | `a/d`=pan, `w/s`=tilt, `h`=홈 (record 방향키와 안 겹침) |

## 홈포즈 조정

종료 시 복귀하는 홈포즈 관절각은 `src/leader_teleop/config.py`의
`HomeReturnConfig`에 있습니다. 내 로봇에 맞는 값은 이렇게 잽니다:

```bash
PYTHONPATH=src python -m leader_teleop.scripts.capture_home_pose \
	--left-arm-port /dev/ttyACM0 --right-arm-port /dev/ttyACM1
```

토크가 풀린 상태에서 원하는 자세로 팔을 잡으면, config에 붙여넣을
값이 출력됩니다.

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

- 실행 전 로봇 주변을 치우세요. `Ctrl+C`를 바로 누를 수 있게 터미널에
  손을 두세요.
- 전압 에러가 반복되면 전원부터 확인하세요. SO-101 리더암은 5V,
  팔로워암은 12V입니다. 데이지체인 커넥터도 다시 꽂아 보세요.
- 리더암과 팔로워암의 좌/우를 바꿔 꽂으면 팔이 교차로 움직입니다.
  실행 직후에는 천천히 움직여 좌/우 대응부터 확인하세요.
